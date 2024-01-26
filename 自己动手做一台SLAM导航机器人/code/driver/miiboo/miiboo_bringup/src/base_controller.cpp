/******************************************************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com

###ROS interface：
1.sub: /cmd_vel
2.pub: /odm , tf(odom->base_footprint)
       /wheel_left_speed , /wheel_right_speed

###serial-com interface：
1.send to serial-com
(1)bps: 115200
(2)data_frame_freq: <200hz
(3)protocol
top  top  sig1    enc1        sig2    enc2        checksum
0xff 0xff 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x??
2.recieve from serial-com
(1)bps: 115200
(2)data_frame_freq:reference by encode_sampling_time
(3)protocol
top  top  sig1    enc1        sig2    enc2        checksum
0xff 0xff 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x?? 0x??

###motor params:
speed_ratio:          (unit: m/encode)    
wheel_distance:       (unit: m)
encode_sampling_time: (unit: s)
*******************************************************************/
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

//serial com
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
//multi thread
#include <pthread.h>
//string
#include <string>
//cos,sin
#include <math.h>

struct ComDev
{
    int SerialCom;
    unsigned char readbuff[11]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char insert_buf;
    unsigned char writebuff[11]={0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char stopbuff[11]={0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    int nread = 0;
    int nwrite = 0;
    bool recv_update_flag=0;
    bool send_update_flag=0;
}myComDev;

struct BaseSensorData
{
    int delta_encode_left; 
    int delta_encode_right; 
}myBaseSensorData;

struct OdomCaculateData
{
    //motor params
    float speed_ratio=0.000176; //unit: m/encode
    float wheel_distance=0.1495; //unit: m
    float encode_sampling_time=0.04; //unit: s
    float cmd_vel_linear_max=0.8; //unit: m/s
    float cmd_vel_angular_max=1.0; //unit: rad/s
    //odom result
    float position_x=0.0; //unit: m
    float position_y=0.0; //unit: m
    float oriention=0.0; //unit: rad
    float velocity_linear=0.0; //unit: m/s
    float velocity_angular=0.0; //unit: rad/s
}myOdomCaculateData;

void ComSetup(const char *COMName)
{
    myComDev.SerialCom = open(COMName, O_RDWR); //set name of serial-com
    if(myComDev.SerialCom == -1)
    {
        printf("Can't open serial port!\n");
    }
    
    struct termios options;
    if(tcgetattr(myComDev.SerialCom, &options) != 0)
    {
        printf("Can't get serial port sets!\n");
    }

    tcflush(myComDev.SerialCom, TCIFLUSH);
    cfsetispeed(&options, B115200);   //set recieve bps of serial-com
    cfsetospeed(&options, B115200);   //set send bps of serial-com
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    if(tcsetattr(myComDev.SerialCom, TCSANOW, &options) != 0)
    {
        printf("Can't set serial port options!\n");
    }
}

//thread: read odom from serial-com
void *myreadframe_thread(void *pt)
{
    while(1)
    {   
        while( (myComDev.nread=read(myComDev.SerialCom,&(myComDev.insert_buf),1))>0 ) //get 1byte by 1byte from serial buffer 
        {
            //debug:print recieved data 1byte by 1byte
            //printf("%x ",myComDev.insert_buf);
          
            //FIFO queue cache
            for(int i=0;i<10;i++)
            {
                myComDev.readbuff[i]=myComDev.readbuff[i+1];
            }
            myComDev.readbuff[10]=myComDev.insert_buf;

            //data analysis
            if(myComDev.readbuff[0]==0xff && myComDev.readbuff[1]==0xff) //top of frame
            {
                //check sum
                unsigned char check_sum=0;
                for(int i=0;i<10;i++)
                  check_sum+=myComDev.readbuff[i];
                if(check_sum==myComDev.readbuff[10])
                {                
                  //debug
                  //printf("recv:\r\n");
                  myComDev.recv_update_flag=0;//clear update flag
                  myBaseSensorData.delta_encode_left=(myComDev.readbuff[2]>0?1:-1)*((myComDev.readbuff[3]<<16)+(myComDev.readbuff[4]<<8)+myComDev.readbuff[5]);
                  myBaseSensorData.delta_encode_right=(myComDev.readbuff[6]>0?1:-1)*((myComDev.readbuff[7]<<16)+(myComDev.readbuff[8]<<8)+myComDev.readbuff[9]);
                  printf("[recv from serial-com]delta_encode_left_feedback=%d delta_encode_right_feedback=%d\r\n",myBaseSensorData.delta_encode_left,myBaseSensorData.delta_encode_right);
                  //###caculate odom###
                  float delta_d_left;
                  float delta_d_right;
                  delta_d_left = (myBaseSensorData.delta_encode_left) * (myOdomCaculateData.speed_ratio);
                  delta_d_right = (myBaseSensorData.delta_encode_right) * (myOdomCaculateData.speed_ratio);
                  float delta_d;
                  float delta_theta;
                  delta_d = (delta_d_left + delta_d_right) * 0.5; //unit: m
                  delta_theta = (delta_d_right - delta_d_left) / (myOdomCaculateData.wheel_distance); //theta axis is anti-clockwise,unit: rad
                  float delta_x;
                  float delta_y;
                  delta_x = delta_d * cos(myOdomCaculateData.oriention + delta_theta*0.5);
                  delta_y = delta_d * sin(myOdomCaculateData.oriention + delta_theta*0.5);
                  //update odom result
                  myOdomCaculateData.position_x += delta_x; //unit: m
                  myOdomCaculateData.position_y += delta_y; //unit: m
                  myOdomCaculateData.oriention += delta_theta; //unit: rad
                  myOdomCaculateData.velocity_linear = delta_d / (myOdomCaculateData.encode_sampling_time); //unit: m/s
                  myOdomCaculateData.velocity_angular = delta_theta / (myOdomCaculateData.encode_sampling_time); //unit: rad/s
                  //#################
                  myComDev.recv_update_flag=1; //set update flag   
                }                
            }
        }//while(..) end
    }//while(1) end
}

//thread: write cmd_vel to serial-com
void *mywriteframe_thread(void *pt)
{
    int i=0;
    //control freq: 100hz (10ms)
    while(1)
    {
        //control
        if(myComDev.send_update_flag==1) //get flag
        {
            myComDev.nwrite=write(myComDev.SerialCom,myComDev.writebuff,11);
            //debug
            //printf("send:%x %x %x %x %x %x\r\n",myComDev.writebuff[0],myComDev.writebuff[1],myComDev.writebuff[2],myComDev.writebuff[3],myComDev.writebuff[4],myComDev.writebuff[5],
            //                                    myComDev.writebuff[6],myComDev.writebuff[7],myComDev.writebuff[8],myComDev.writebuff[9],myComDev.writebuff[10]);
            myComDev.send_update_flag=0; //clear flag
            i=0; //clear stop count
        }
        else if(i==50) //if not input cmd_vel during 0.5s, stop motor
        {
            //stop
            myComDev.nwrite=write(myComDev.SerialCom,myComDev.stopbuff,11);
        }

        if(i>=50)
            i=0;
        else
            i++;

        ros::Duration(0.01).sleep(); //delay 10ms
    }
}

void callback(const geometry_msgs::Twist & cmd_input)
{
    float angular_temp;
    float linear_temp;
    linear_temp = cmd_input.linear.x ;//m/s
    angular_temp = cmd_input.angular.z ;//rad/s

    //motor max vel limit
    float linear_max_limit = myOdomCaculateData.cmd_vel_linear_max;
    float angular_max_limit = myOdomCaculateData.cmd_vel_angular_max;
    if(linear_temp>linear_max_limit)
        linear_temp = linear_max_limit;
    if(linear_temp<(-1*linear_max_limit))
        linear_temp = -1*linear_max_limit;
    if(angular_temp>angular_max_limit)
        angular_temp = angular_max_limit;
    if(angular_temp<(-1*angular_max_limit))
        angular_temp = -1*angular_max_limit;

    int delta_encode_left_temp;
    int delta_encode_right_temp;
    delta_encode_left_temp = (linear_temp-0.5*(myOdomCaculateData.wheel_distance)*angular_temp)*(myOdomCaculateData.encode_sampling_time)/(myOdomCaculateData.speed_ratio);
    delta_encode_right_temp = (linear_temp+0.5*(myOdomCaculateData.wheel_distance)*angular_temp)*(myOdomCaculateData.encode_sampling_time)/(myOdomCaculateData.speed_ratio);
 
    while(myComDev.send_update_flag!=0);//wait for flag clear
    printf("[send to serial-com]delta_encode_left_target=%d delta_encode_right_target=%d\r\n",delta_encode_left_temp,delta_encode_right_temp);
    //left motor enc set
    if(delta_encode_left_temp>=0)
        myComDev.writebuff[2] = 0x01;
    else
        myComDev.writebuff[2] = 0x00;
    myComDev.writebuff[3] = abs(delta_encode_left_temp)>>16;
    myComDev.writebuff[4] = (abs(delta_encode_left_temp)>>8)&0xff;
    myComDev.writebuff[5] = abs(delta_encode_left_temp)&0xff;
    //right motor enc set
    if(delta_encode_right_temp>=0)
        myComDev.writebuff[6] = 0x01;
    else
        myComDev.writebuff[6] = 0x00;
    myComDev.writebuff[7] = abs(delta_encode_right_temp)>>16;
    myComDev.writebuff[8] = (abs(delta_encode_right_temp)>>8)&0xff;
    myComDev.writebuff[9] = abs(delta_encode_right_temp)&0xff;
    //create checksum
    myComDev.writebuff[10]=myComDev.writebuff[0]+myComDev.writebuff[1]+myComDev.writebuff[2]+myComDev.writebuff[3]+myComDev.writebuff[4]+
                           myComDev.writebuff[5]+myComDev.writebuff[6]+myComDev.writebuff[7]+myComDev.writebuff[8]+myComDev.writebuff[9];
    myComDev.send_update_flag=1; //set flag
}

int main(int argc, char **argv)
{
  /******************************************************************
  ## 关于
  * 作者: （英文名）xiihoo（中文名）张虎（网名）  小虎哥哥爱学习
  * 官网:  http://www.xiihoo.com
  * QQ群： 
    + QQ技术1群：728661815（1群已满，请加3群）
    + QQ技术2群：117698356（2群已满，请加3群）
    + QQ技术3群：891252940
  * 微信:  robot4xiihoo
  * 微信公众号: 小虎哥哥爱学习
  * 邮箱:  robot4xiihoo@163.com
  * 源码:  https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
  * 淘宝:  https://xiihoo.taobao.com
  * B站:   https://space.bilibili.com/66815220
  ## 资料汇总下载
  * 百度网盘链接： https://pan.baidu.com/s/1nHbI0mi-iM72NAcQlAU1uQ?pwd=1234
  * 提取码：1234
  ## 目录
  * 第一章：Linux基础
  * 第二章：ROS入门
  * 第三章：感知与大脑
  * 第四章：差分底盘设计
  * 第五章：树莓派3开发环境搭建
  * 第六章：SLAM建图与自主避障导航
  * 第七章：语音交互与自然语言处理
  * 附录A：用于ROS机器人交互的Android手机APP开发
  * 附录B：用于ROS机器人管理调度的后台服务器搭建
  * 附录C：如何选择ROS机器人平台进行SLAM导航入门
  ## 环境要求
  * ubuntu 16.04 或 ubuntu-mate 16.04
  * ROS kinetic
  *******************************************************************/
  ros::init(argc, argv, "miiboo_bringup_node");
  ros::NodeHandle n;

  std::string com_port = "/dev/ttyUSB0";
  std::string cmd_vel_topic = "cmd_vel";
  std::string odom_pub_topic = "odom";
  std::string wheel_left_speed_pub_topic = "wheel_left_speed";
  std::string wheel_right_speed_pub_topic = "wheel_right_speed";
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_footprint";
  
  //get param from user launch
  /*serial_com set*/
  ros::param::get("~com_port", com_port);
  /*motor param set*/
  ros::param::get("~speed_ratio", myOdomCaculateData.speed_ratio);
  ros::param::get("~wheel_distance", myOdomCaculateData.wheel_distance);
  ros::param::get("~encode_sampling_time", myOdomCaculateData.encode_sampling_time);
  /*velocity limit*/
  ros::param::get("~cmd_vel_linear_max", myOdomCaculateData.cmd_vel_linear_max);
  ros::param::get("~cmd_vel_angular_max", myOdomCaculateData.cmd_vel_angular_max);
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~wheel_left_speed_pub_topic", wheel_left_speed_pub_topic);
  ros::param::get("~wheel_right_speed_pub_topic", wheel_right_speed_pub_topic);
  ros::param::get("~odom_frame_id", odom_frame_id);
  ros::param::get("~odom_child_frame_id", odom_child_frame_id);

  ros::Subscriber sub = n.subscribe(cmd_vel_topic, 20, callback);
  ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
  ros::Publisher wheel_left_speed_pub= n.advertise<std_msgs::Float32>(wheel_left_speed_pub_topic, 20);
  ros::Publisher wheel_right_speed_pub= n.advertise<std_msgs::Float32>(wheel_right_speed_pub_topic, 20);
  
  //open deviece file of serial-com(such as /dev/ttyUSB*) 
  ComSetup(com_port.c_str());
  //create thread of serial-com read  
  pthread_t id_1;
  int ret1=pthread_create(&id_1,NULL,*myreadframe_thread,NULL);
  //create thread of serial-com write  
  pthread_t id_2;
  int ret2=pthread_create(&id_2,NULL,*mywriteframe_thread,NULL);

  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  std_msgs::Float32 wheel_left_speed_msg;
  std_msgs::Float32 wheel_right_speed_msg;
  geometry_msgs::Quaternion odom_quat;
  //covariance matrix
  float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
  //load covariance matrix
  for(int i = 0; i < 36; i++)
  {
      odom.pose.covariance[i] = covariance[i];;
  }       

  ros::Rate loop_rate(10.0); //10.0HZ
  while(ros::ok())
  {
      if(myComDev.recv_update_flag==1)
      {   
            //odom_oriention trans to odom_quat
            odom_quat = tf::createQuaternionMsgFromYaw(myOdomCaculateData.oriention);//yaw trans quat

            //pub tf(odom->base_footprint)
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = odom_frame_id;     
            odom_trans.child_frame_id = odom_child_frame_id;       
            odom_trans.transform.translation.x = myOdomCaculateData.position_x;
            odom_trans.transform.translation.y = myOdomCaculateData.position_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;        
            //pub odom
            odom.header.stamp = ros::Time::now(); 
            odom.header.frame_id = odom_frame_id;
            odom.child_frame_id = odom_child_frame_id;       
            odom.pose.pose.position.x = myOdomCaculateData.position_x;     
            odom.pose.pose.position.y = myOdomCaculateData.position_y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;       
            odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
            odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
            //pub enc
            wheel_left_speed_msg.data = (myBaseSensorData.delta_encode_left)*(myOdomCaculateData.speed_ratio)/(myOdomCaculateData.encode_sampling_time);
            wheel_right_speed_msg.data = (myBaseSensorData.delta_encode_right)*(myOdomCaculateData.speed_ratio)/(myOdomCaculateData.encode_sampling_time);

            odom_broadcaster.sendTransform(odom_trans);
            odom_pub.publish(odom);
            wheel_left_speed_pub.publish(wheel_left_speed_msg);
            wheel_right_speed_pub.publish(wheel_right_speed_msg);
      }

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

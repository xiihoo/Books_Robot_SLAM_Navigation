/******************************************************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com

###serial-com interface:
1.send to serial-com
(1)bps: 115200
(2)data_frame_freq: <200hz
(3)protocol
top  top  sig_Kp       Kp       sig_Ki       Ki       sig_Kd       Kd       checksum
0xff 0xff 0x??   0x?? 0x?? 0x?? 0x??   0x?? 0x?? 0x?? 0x??   0x?? 0x?? 0x?? 0x??
(
case1:version and pid_params request(0x00 0x00 ... 0x00)
case2:reset pid_params to default request(0xff 0xff ... 0xff)
case3:set pid_params(0x?? 0x?? ... 0x??)
)
2.recieve from serial-com
(1)bps: 115200
(2)data_frame_freq: none
(3)protocol
char printf() one-by-one

###user interface:
select cmd mode:
1 version and pid_params request
2 reset pid_params to default request
3 set pid_params
please input number 1 2 or 3:

please input kp ki kd:
*******************************************************************/
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

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
//std::cin
#include <iostream>

struct ComDev
{
    int SerialCom;
    unsigned char insert_buf;
    unsigned char writebuff[15]={0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int nread = 0;
    int nwrite = 0;
}myComDev;

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

//thread: read from serial-com
void *myreadframe_thread(void *pt)
{
    while(1)
    {   
        while( (myComDev.nread=read(myComDev.SerialCom,&(myComDev.insert_buf),1))>0 ) //get 1byte by 1byte from serial buffer 
        {
            printf("%c",myComDev.insert_buf);
                     
        }//while(..) end
    }//while(1) end
}

//thread: write to serial-com
void *mywriteframe_thread(void *pt)
{
    float kp_set,ki_set,kd_set;
    int cmd_num;
    while(1)
    {
      //cin
      ros::Duration(1.0).sleep();
      std::cout<<"select cmd mode:"<<std::endl;
      std::cout<<"1 version and pid_params request"<<std::endl;
      std::cout<<"2 reset pid_params to default request"<<std::endl;
      std::cout<<"3 set pid_params"<<std::endl;
      std::cout<<"please input number 1 2 or 3:"<<std::endl;
      std::cin>>cmd_num;

      if(cmd_num==1)
      {
        myComDev.writebuff[2] = 0x00;
        myComDev.writebuff[3] = 0x00;
        myComDev.writebuff[4] = 0x00;
        myComDev.writebuff[5] = 0x00;
        myComDev.writebuff[6] = 0x00;
        myComDev.writebuff[7] = 0x00;
        myComDev.writebuff[8] = 0x00;
        myComDev.writebuff[9] = 0x00;
        myComDev.writebuff[10] = 0x00;
        myComDev.writebuff[11] = 0x00;
        myComDev.writebuff[12] = 0x00;
        myComDev.writebuff[13] = 0x00;
      }
      else if(cmd_num==2)
      {
        myComDev.writebuff[2] = 0xff;
        myComDev.writebuff[3] = 0xff;
        myComDev.writebuff[4] = 0xff;
        myComDev.writebuff[5] = 0xff;
        myComDev.writebuff[6] = 0xff;
        myComDev.writebuff[7] = 0xff;
        myComDev.writebuff[8] = 0xff;
        myComDev.writebuff[9] = 0xff;
        myComDev.writebuff[10] = 0xff;
        myComDev.writebuff[11] = 0xff;
        myComDev.writebuff[12] = 0xff;
        myComDev.writebuff[13] = 0xff;
      }
      else if(cmd_num==3)
      {
        std::cout<<"please input kp ki kd:"<<std::endl;
        std::cin>>kp_set>>ki_set>>kd_set;
        //Kp
        if(kp_set>=0)
          myComDev.writebuff[2] = 0x01;
        else
          myComDev.writebuff[2] = 0x00;
        myComDev.writebuff[3] = abs(int(kp_set*10000))>>16;
        myComDev.writebuff[4] = (abs(int(kp_set*10000))>>8)&0xff;
        myComDev.writebuff[5] = abs(int(kp_set*10000))&0xff; 
        //Ki
        if(ki_set>=0)
          myComDev.writebuff[6] = 0x01;
        else
          myComDev.writebuff[6] = 0x00;
        myComDev.writebuff[7] = abs(int(ki_set*10000))>>16;
        myComDev.writebuff[8] = (abs(int(ki_set*10000))>>8)&0xff;
        myComDev.writebuff[9] = abs(int(ki_set*10000))&0xff; 
        //Kd
        if(kd_set>=0)
          myComDev.writebuff[10] = 0x01;
        else
          myComDev.writebuff[10] = 0x00;
        myComDev.writebuff[11] = abs(int(kd_set*10000))>>16;
        myComDev.writebuff[12] = (abs(int(kd_set*10000))>>8)&0xff;
        myComDev.writebuff[13] = abs(int(kd_set*10000))&0xff;
      }
      else
      {
        continue;
      }   
      //create checksum
      myComDev.writebuff[14]=myComDev.writebuff[0]+myComDev.writebuff[1]+myComDev.writebuff[2]+myComDev.writebuff[3]+myComDev.writebuff[4]+
                           myComDev.writebuff[5]+myComDev.writebuff[6]+myComDev.writebuff[7]+myComDev.writebuff[8]+myComDev.writebuff[9]+
                           myComDev.writebuff[10]+myComDev.writebuff[11]+myComDev.writebuff[12]+myComDev.writebuff[13]; 
      
      //execute send
      myComDev.nwrite=write(myComDev.SerialCom,myComDev.writebuff,15);
    }
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
  ros::init(argc, argv, "pid_set_node");
  ros::NodeHandle n;

  std::string com_port = "/dev/ttyUSB1";

  ros::param::get("~com_port", com_port);
 
  //open deviece file of serial-com(such as /dev/ttyUSB*) 
  ComSetup(com_port.c_str());
  //create thread of serial-com read  
  pthread_t id_1;
  int ret1=pthread_create(&id_1,NULL,*myreadframe_thread,NULL);
  //create thread of serial-com write  
  pthread_t id_2;
  int ret2=pthread_create(&id_2,NULL,*mywriteframe_thread,NULL);

  while(ros::ok()){}

  return 0;
}

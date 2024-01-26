/*
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com

HI229出厂默认输出协议接收:
输出 sum = 41
0x5A+0xA5+LEN_LOW+LEN_HIGH+CRC_LOW+CRC_HIGH+ 0x90+ID(1字节) + 0xA0+Acc(加速度6字节) + 0xB0+Gyo(角速度6字节) + 0xC0+Mag(地磁6字节) + 0xD0 +AtdE(欧拉角6字节PRY) + 0xF0+Pressure(压力4字节)
*/
/*
输出 sum = 41+16+1=58
0x5A+0xA5+LEN_LOW+LEN_HIGH+CRC_LOW+CRC_HIGH+ 0x90+ID(1字节) + 0xA0+Acc(加速度6字节) + 0xB0+Gyo(角速度6字节) + 0xC0+Mag(地磁6字节) + 0xD0 +AtdE(欧拉角6字节PRY) 
+ 0xD1+quat(四元数16字节WXYZ) 0xF0+Pressure(压力4字节)
*/
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf/transform_broadcaster.h"

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

using namespace std;
using namespace boost::asio;


#define MAX_PACKET_LEN          (58)// length of the data

typedef enum
{
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemIPAdress =             0x92,   /* ip address           size: 4 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemAccRawFiltered =       0xA1,
    kItemAccDynamic =           0xA2,
    kItemGyoRaw =               0xB0,   /* raw gyro             size: 3x2 */
    kItemGyoRawFiltered =       0xB1,
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemMagRawFiltered =       0xC1,
    kItemAtdE =                 0xD0,   /* eular angle          size:3x2 */
    kItemAtdQ =                 0xD1,   /* att q,               size:4x4 */
    kItemTemp =                 0xE0,
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0xFF,
}ItemID_t;

uint8_t ID;
int16_t AccRaw[3];
int16_t GyoRaw[3];
int16_t MagRaw[3];
float Eular[3];
float pith,roll,yaw;
float quat[4];
int32_t Pressure;

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
    ros::init(argc, argv, "imu_read_node");
    ros::NodeHandle n;

    string com_port = "/dev/ttyUSB0";
    string imu_frame_id = "imu_link";
    string mag_frame_id = "imu_link";
    string imu_topic = "/imu";
    string mag_topic = "/mag";
    ros::param::get("~com_port",com_port);
    ros::param::get("~imu_frame_id",imu_frame_id);
    ros::param::get("~mag_frame_id",mag_frame_id);
    ros::param::get("~imu_topic",imu_topic);
    ros::param::get("~mag_topic",mag_topic);

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>(mag_topic, 1000);

    io_service iosev;
    serial_port sp(iosev, com_port);
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    int count = 0;
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        // 向串口写数据
        // write(sp, buffer("Hello world", 12));

        // 向串口读数据
        uint8_t buf_tmp[1];
        uint8_t buf[MAX_PACKET_LEN-1];
        read(sp, buffer(buf_tmp));
        if(buf_tmp[0] == 0x5A )
        {
            read(sp, buffer(buf));

            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.seq = count;
            imu_msg.header.frame_id =  imu_frame_id;

            sensor_msgs::MagneticField mag_msg;
            mag_msg.header.stamp = ros::Time::now();    
            mag_msg.header.seq = count;
            mag_msg.header.frame_id =  mag_frame_id;    

            /* 
            按出厂默认输出协议接收:
            0x5A+0xA5+LEN_LOW+LEN_HIGH+CRC_LOW+CRC_HIGH+ 0x90+ID(1字节) + 0xA0+Acc(加速度6字节) + 0xB0+Gyo(角速度6字节) + 0xC0+Mag(地磁6字节) + 0xD0 +AtdE(欧拉角6字节) + 0xF0+Pressure(压力4字节)
            */
            int i=0;
            if(buf[i] == 0xA5) /* user ID */
            {
                               
                i+=5;//moving right 5bit to 0x90

                //user ID
                if(buf[i+0] == kItemID) 
                {
                    ID = buf[i+1];
                }
                //Acc value
                if(buf[i+2] == kItemAccRaw)
                {
                    memcpy(AccRaw, &buf[i+3], 6);
                    imu_msg.linear_acceleration.x =AccRaw[0]/1000.0 * 9.7887;//unit:m/s^2
                    imu_msg.linear_acceleration.y =AccRaw[1]/1000.0 * 9.7887;//unit:m/s^2
                    imu_msg.linear_acceleration.z =AccRaw[2]/1000.0 * 9.7887;//unit:m/s^2
                }
                //Gyro value
                if(buf[i+9] == kItemGyoRaw)
                {
                    memcpy(GyoRaw, &buf[i+10], 6);
                    imu_msg.angular_velocity.x = GyoRaw[0]*M_PI/10.0/180.0;//unit:rad/s
                    imu_msg.angular_velocity.y = GyoRaw[1]*M_PI/10.0/180.0;//unit:rad/s
                    imu_msg.angular_velocity.z = GyoRaw[2]*M_PI/10.0/180.0;//unit:rad/s
                }
                //Mag value
                if(buf[i+16] == kItemMagRaw)
                {
                    memcpy(MagRaw, &buf[i+17], 6);
                    mag_msg.magnetic_field.x = MagRaw[0]/1000.0/10000.0; //unit:Tesla
                    mag_msg.magnetic_field.y = MagRaw[1]/1000.0/10000.0; //unit:Tesla
                    mag_msg.magnetic_field.z = MagRaw[2]/1000.0/10000.0; //unit:Tesla
                }
                //atd E
                if(buf[i+23] == kItemAtdE)
                {
                    Eular[0] = ((float)(int16_t)(buf[i+24] + (buf[i+25]<<8)))/100;
                    Eular[1] = ((float)(int16_t)(buf[i+26] + (buf[i+27]<<8)))/100;
                    Eular[2] = ((float)(int16_t)(buf[i+28] + (buf[i+29]<<8)))/10;
	                //fixed error by 2021/04/23
		            pith = Eular[0];//unit:°
		            roll = Eular[1];//unit:°
		            yaw = Eular[2];//unit:°
                    //geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pith, yaw);
                    //imu_msg.orientation = quat;
                    //imu_msg.linear_acceleration_covariance=boost::array<double, 9>
                }
                //atd Q
                if(buf[i+30] == kItemAtdQ)
                {
                    memcpy(quat, &buf[i+31], 16);
                    imu_msg.orientation.x = quat[1];
                    imu_msg.orientation.y = quat[2];
                    imu_msg.orientation.z = quat[3];
                    imu_msg.orientation.w = quat[0];
                }
                /*
                //Pressure value
                if(buf[i+30] == kItemPressure)
                {
                    memcpy(&Pressure, &buf[i+31], 4);
                }
                */

                //debug
                /*
                printf("ID: %d \r\n", ID);
                printf("AccRaw: %d %d %d\r\n", AccRaw[0], AccRaw[1], AccRaw[2]);
                printf("GyoRaw: %f %f %f\r\n", GyoRaw[0], GyoRaw[1], GyoRaw[2]);
                printf("MagRaw: %d %d %d\r\n", MagRaw[0], MagRaw[1], MagRaw[2]);
                printf("Eular: roll=%0.2f pith=%0.2f yaw=%0.2f\r\n", roll, pith, yaw;
                printf("Pressure: %d Pa\r\n\n", Pressure);
                */

                imu_pub.publish(imu_msg);
                mag_pub.publish(mag_msg);

                //ros::spinOnce();
                //loop_rate.sleep();

                count++;
            }
        }
    }//while end

    iosev.run();
    return 0;
}

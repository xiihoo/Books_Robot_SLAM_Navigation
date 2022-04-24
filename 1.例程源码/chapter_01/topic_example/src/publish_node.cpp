/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include "ros/ros.h" 
#include "std_msgs/String.h" 

#include <sstream>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "publish_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok()) 
  {
    std_msgs::String msg;

    std::stringstream ss; 
    ss << "hello " << count; 
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
  
    chatter_pub.publish(msg);
  
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

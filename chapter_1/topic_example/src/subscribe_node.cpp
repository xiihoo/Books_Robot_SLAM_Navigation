/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include "ros/ros.h" 
#include "std_msgs/String.h" 

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]",msg->data.c_str());
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "subscribe_node");
  ros::NodeHandle nh;

  ros::Subscriber chatter_sub = nh.subscribe("chatter", 1000,chatterCallback);

  ros::spin();

  return 0;
}

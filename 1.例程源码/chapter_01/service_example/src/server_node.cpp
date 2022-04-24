/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include "ros/ros.h"
#include "service_example/AddTwoInts.h"

bool add_execute(service_example::AddTwoInts::Request &req,
service_example::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("recieve request: a=%ld,b=%ld",(long int)req.a,(long int)req.b);
  ROS_INFO("send response: sum=%ld",(long int)res.sum);
  return true;
} 

int main(int argc,char **argv)
{
  ros::init(argc,argv,"server_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("add_two_ints",add_execute);
  ROS_INFO("service is ready!!!");
  ros::spin();

  return 0;
}

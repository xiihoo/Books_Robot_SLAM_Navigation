/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include "ros/ros.h"
#include "service_example/AddTwoInts.h"

#include <iostream>

int main(int argc,char **argv)
{
  ros::init(argc,argv,"client_node");
  ros::NodeHandle nh;

  ros::ServiceClient client =
  nh.serviceClient<service_example::AddTwoInts>("add_two_ints");
  service_example::AddTwoInts srv;
  
  while(ros::ok())
  {
    long int a_in,b_in;
    std::cout<<"please input a and b:";
    std::cin>>a_in>>b_in;

    srv.request.a = a_in;
    srv.request.b = b_in;
    if(client.call(srv))
    {
      ROS_INFO("sum=%ld",(long int)srv.response.sum);
    }
    else
    {
      ROS_INFO("failed to call service add_two_ints");
    }
  }
  return 0;
}

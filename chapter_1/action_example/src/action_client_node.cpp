/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "action_example/CountDownAction.h"

void doneCB(const actionlib::SimpleClientGoalState& state, 
            const action_example::CountDownResultConstPtr& result)
{
    ROS_INFO("done");
    ros::shutdown();
}
void activeCB()
{
    ROS_INFO("active");
}
void feedbackCB(const action_example::CountDownFeedbackConstPtr& feedback)
{
    ROS_INFO("feedback:[%f,%d]",feedback->count_percent,feedback->count_current);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client_node");

    actionlib::SimpleActionClient<action_example::CountDownAction> ac("/count_down",true);

    ROS_INFO("wait for action server to start!");
    ac.waitForServer();

    action_example::CountDownGoal goal;
    std::cout<<"please input target_number and target_step:"<<std::endl;
    std::cin>>goal.target_number>>goal.target_step;

    ac.sendGoal(goal,&doneCB,&activeCB,&feedbackCB);

    ros::spin();
    return 0;
}

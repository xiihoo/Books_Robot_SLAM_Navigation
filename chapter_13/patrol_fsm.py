#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints =[
    ['one', (-0.2, -2.1), (0.0, 0.0, 0.0, 1.0)],
    ['two', (0.4, -1.3), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    ['three', (0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]

#################################################################
# Author :  xiihoo
# Website:  www.xiihoo.com
# E-mail :  robot4xiihoo@163.com
# Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
#################################################################
if __name__ == '__main__':
    rospy.init_node('patrol')
    patrol = StateMachine(['succeeded','aborted','preempted'])
    with patrol:
        for i,w in enumerate(waypoints):
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = w[1][0]
            goal_pose.target_pose.pose.position.y = w[1][1]
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = w[2][0]
            goal_pose.target_pose.pose.orientation.y = w[2][1]
            goal_pose.target_pose.pose.orientation.z = w[2][2]
            goal_pose.target_pose.pose.orientation.w = w[2][3]

            StateMachine.add(
                w[0],
                SimpleActionState('move_base', MoveBaseAction, goal=goal_pose),
                transitions={'succeeded':waypoints[(i + 1) % len(waypoints)][0]}
            )
            
    patrol.execute()
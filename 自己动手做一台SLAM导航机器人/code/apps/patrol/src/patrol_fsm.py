#!/usr/bin/env python

'''
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
'''

import rospy 
from smach import StateMachine  
from smach_ros import SimpleActionState  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints =[    
    ['one', (-0.2, -2.1), (0.0, 0.0, 0.0, 1.0)],    
    ['two', (0.4, -1.3), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    ['three', (0.0, 0.0), (0.0, 0.0, 0.0, 1.0)] 
]

if __name__ == '__main__':
    '''
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
    '''
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

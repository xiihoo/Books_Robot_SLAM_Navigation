[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com



#miiboo
#***********************#
@version:V1.0.0
@date   :2018/04/17 11:08 
@author :hiramzhang
@log:
1.send stop when never input cmd_vel during 2.0s,correct delay function bug.
2.correct turn direct bug,delta_theta=delta_d_riht-delta_d_left.
3.add calibration scripts:check_linear.py, check_angular.py
4.add teleop_twist_keyboard.py
#***********************#
@version:V1.0.1
@date   :2018/04/17 22:06
@author :hiramzhang
@log:
1.get a good calibration:speed_ratio=0.000176,wheel_distance=0.1495
#***********************#
@version:V1.0.2
@date   :2018/06/29 12:07
@author :hiramzhang
@log:
1.add miiboo_description
#***********************#
@version:V1.0.3
@date   :2018/08/28 18:04
@author :hiramzhang
@log:
1.edit miiboo_bringup for new motor of miiboo_stm32.
2.add  pid_set node to debug pid params by usart1.
#***********************#
@version:V2.0.0
@date   :2018/09/07 17:44
@author :hiramzhang
@log:
1.add many ros params for user config.
2.mapping and navigation in new base is ok.
#***********************#
@version:V2.0.1
@date   :2018/09/12 17:37
@author :hiramzhang
@log:
1.accuracy of pid_set improve to 0.0001
#***********************#
@version:V2.0.2
@date   :2018/09/14 17:42
@author :hiramzhang
@log:
1.add velocity limit in ros params.
2.add calib vel_linear in params.
3.add calib vel_angular in params.
#***********************#
@version:V3.0.0
@date   :2019/01/22 18:59
@author :hiramzhang
@log:
1.pub real speed of wheel to topic(/wheel_left_speed,/wheel_right_speed).
2.support 1,2,3 cmd in pid_set.cpp
#***********************#
@version:V3.0.1
@date   :2019/01/28 22:54
@author :hiramzhang
@log:
1.calib miiboo is ok.
2.set new urdf for miiboo.
#***********************#

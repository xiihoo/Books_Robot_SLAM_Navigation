# DIY_A_SLAM_Navigation_Robot
# 自己动手做一台SLAM导航机器人
## 源码仓库地址
* https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
* https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
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
## 源码说明
本仓库的code文件夹中包含4个子文件夹，分别为driver、carto、nav和apps；其中文件夹driver中存放的是miiboo机器人相关的底层ROS驱动包源码，文件夹carto中存放的是google开源SLAM算法（即cartographer）相关的核心算法库以及ROS接口调用项目的源码，文件夹nav中存放的是导航相关的核心算法库、插件以及ROS接口调用项目的源码，文件夹apps中存放的是用户应用层业务逻辑代码。
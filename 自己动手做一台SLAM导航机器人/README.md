# DIY_A_SLAM_Navigation_Robot
# 自己动手做一台SLAM导航机器人
## 温馨提示
* 本仓库用于同步存放我在知乎专栏《自己动手做一台SLAM导航机器人》发布的系列文章的pdf版本和code源码：
  + https://www.zhihu.com/column/c_1084087088789569536
* 基于代号为“miiboo”的机器人项目《自己动手做一台SLAM导航机器人》已经完结。而基于新代号“xiihoo”的机器人项目《机器人SLAM导航：核心技术与实战》正是在项目《自己动手做一台SLAM导航机器人》的基础上衍生而来，今后将主要维护这个新项目，该新项目详情如下：
  + https://github.com/xiihoo/Books_Robot_SLAM_Navigation
  + https://gitee.com/xiihoo-robot/Books_Robot_SLAM_Navigation
  + https://item.jd.com/13041503.html
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
* 源码GitHub:  https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
* 源码Gitee(访问更快):  https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
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
## 源码说明
本仓库的code文件夹中包含4个子文件夹，分别为driver、carto、nav和apps；其中文件夹driver中存放的是miiboo机器人相关的底层ROS驱动包源码，文件夹carto中存放的是google开源SLAM算法（即cartographer）相关的核心算法库以及ROS接口调用项目的源码，文件夹nav中存放的是导航相关的核心算法库、插件以及ROS接口调用项目的源码，文件夹apps中存放的是用户应用层业务逻辑代码。
<table>
  <tr>
    <th align="center">文件夹</th>
    <th align="center">源码包</th>
    <th align="center">说明</th>
  </tr>
  <tr>
    <td align="center" rowspan=5>driver</td>
    <td align="left">miiboo</td>
    <td align="left">机器人底盘驱动（电机控制和URDF模型）</td>
  </tr>
  <tr>
    <td align="left">miiboo_imu</td>
    <td align="left">IMU传感器驱动</td>
  </tr>
  <tr>
    <td align="left">usb_cam</td>
    <td align="left">USB摄像头驱动</td>
  </tr>
  <tr>
    <td align="left">ydlidar</td>
    <td align="left">激光雷达驱动</td>
  </tr>
  <tr>
    <td align="left">broadcast_ip</td>
    <td align="left">广播本机IP地址</td>
  </tr>   
  <tr>
    <td align="center" rowspan=3>carto</td>
    <td align="left">cartographer_ros</td>
    <td align="left">cartographer算法的ROS接口调用</td>
  </tr>
  <tr>
    <td align="left">cartographer</td>
    <td align="left">cartographer算法的核心库</td>
  </tr>
  <tr>
    <td align="left">ceres-solver</td>
    <td align="left">cartographer算法中需要调用的非线性优化库</td>
  </tr>
  <tr>
    <td align="center" rowspan=3>nav</td>
    <td align="left">navigation-kinetic-devel</td>
    <td align="left">导航功能包集</td>
  </tr>
  <tr>
    <td align="left">teb_local_planner</td>
    <td align="left">teb路径规划插件</td>
  </tr>
  <tr>
    <td align="left">miiboo_nav</td>
    <td align="left">导航算法的ROS接口调用</td>
  </tr>
  <tr>
    <td align="center" rowspan=2>apps</td>
    <td align="left">miiboo_asr</td>
    <td align="left">语音交互应用程序</td>
  </tr>
  <tr>
    <td align="left">patrol</td>
    <td align="left">指定路线巡航应用程序</td>
  </tr>
</table>

## 源码编译
在正式编译本仓库内的代码之前，你需要先搭建好“miiboo”机器人的软硬件环境。硬件环境是指机器人的传感器（电机控制板、激光雷达、IMU、摄像头等）以及主机（比如树莓派、Jetson nano/tx1/tx2、RK3399等开发板）；软件环境是指机器人所搭载主机的操作系统（也就是ubuntu 16.04 或 ubuntu-mate 16.04）以及ROS系统（也就是ROS kinetic）。
* 硬件环境搭建教程：
  + 《自己动手做一台SLAM导航机器人》第三章：感知与大脑
  + 《自己动手做一台SLAM导航机器人》第四章：差分底盘设计
* 软件环境搭建教程：
  + 《自己动手做一台SLAM导航机器人》第一章：Linux基础
  + 《自己动手做一台SLAM导航机器人》第二章：ROS入门
  + 《自己动手做一台SLAM导航机器人》第五章：树莓派3开发环境搭建

如果“miiboo”机器人的软硬件环境已经搭建完毕，就可以将本仓库 https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot 或 https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot 的代码下载或者克隆到你自己的计算机，并且将下载或克隆下来的所有文件拷贝到你的机器人主机备用，其实只需要将文件夹code中的内容拷贝到你的机器人主机。
由于code文件夹下的driver、carto、nav和apps都是ROS功能包或ROS功能包集，所以不能直接编译，而是需要放在特定的ROS工作空间才能编译。
### driver源码编译
请按教程（《自己动手做一台SLAM导航机器人》第二章：ROS入门）中的方法在机器人主机端新建一个catkin_ws工作空间，然后将本项目提供的"code/driver/"路径中的所有功能包拷贝到该工作空间的"catkin_ws/src/"路径中，最后使用catkin_make命令编译即可。
### carto源码编译
由于cartographer算法采用catkin_make_isolated命令编译，这与传统的catkin_make命令编译有所不同。因此这里要为cartographer算法专门新建一个catkin_ws_carto工作空间，然后按照cartographer官方教程步骤或者本项目的教程（《自己动手做一台SLAM导航机器人》第六章：SLAM建图与自主避障导航）将cartographer算法安装到catkin_ws_carto工作空间。由于实际使用中需要对cartographer算法的部分代码以及配置文件进行修改，所以大家需要将本项目提供的"code/carto/"路径中的源码覆盖到上面默认源码安装路径“catkin_ws_carto/src/”。源码覆盖完成后，使用catkin_make_isolated命令重新编译即可生效。
### nav源码编译
由于nav里面包含的功能包很多，为了便于管理维护。这里同样为其专门新建一个catkin_ws_nav工作空间，接着将本项目提供的"code/nav/"路径中的所有功能包拷贝到该工作空间的"catkin_ws_nav/src/"路径中，然后按照本项目的教程（《自己动手做一台SLAM导航机器人》第六章：SLAM建图与自主避障导航）进行编译。
### apps源码编译
同样为了便于管理维护，这里为apps里面包含的功能包专门新建一个catkin_ws_apps工作空间。接着将本项目提供的"code/apps/"路径中的所有功能包拷贝到该工作空间的"catkin_ws_apps/src/"路径中，然后按照本项目的教程（《自己动手做一台SLAM导航机器人》第六章：SLAM建图与自主避障导航、第七章：语音交互与自然语言处理）进行编译。

## 程序运行
当所以源码都编译完成后，就可以按照需求顺序启动需要的功能包程序。比如要进行SLAM建图，就先启动driver里面的各个功能包，然后启动carto里面的功能包，最后用遥控或键盘控制机器人移动建图并将建好的地图保存下来。如果要进行导航，就先启动driver里面的各个功能包，然后启动nav里面的功能包,最后通过远程电脑端的rviz或手机APP给机器人发送目标点。如果要实时运行SLAM建图和导航，就先启动driver里面的各个功能包，然后启动carto里面的功能包，接着启动nav里面的功能包，最后通过远程电脑端的rviz或手机APP给机器人发送目标点，当不发送目标点时也可以用遥控或键盘控制机器人移动建图，不过这种同时启动SLAM建图和导航的情况需要进行特殊的配置比较麻烦（新手不推荐）。如果要进行指定路线巡航，就需要先建立好地图或在导航过程中实时建图，然后启动导航，最后启动apps里面对应的应用层功能包。
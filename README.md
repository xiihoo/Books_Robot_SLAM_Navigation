# 《机器人SLAM导航核心技术与实战》张虎 著 （随书源码+课件+习题答案）
## 温馨提示
* 配套源码中存在一些bug(主要是各种库版本冲突和配置文件方面的兼容问题)的部分代码还未上传，我正在抓紧修复之后会尽快上传，大家有疑问也可以直接联系我。
* 基于代号为“miiboo”的机器人项目《自己动手做一台SLAM导航机器人》已经完结。而基于新代号“xiihoo”的机器人项目《机器人SLAM导航：核心技术与实战》正是在项目《自己动手做一台SLAM导航机器人》的基础上衍生而来，今后将主要维护这个xiihoo机器人新项目。而关于旧项目miiboo机器人，想了解详情的话请移步如下链接地址：
  + https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
  + https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
## 关于
* 作者: （英文名）xiihoo（中文名）张虎（网名）  小虎哥哥爱学习
* 官网:  http://www.xiihoo.com
* QQ群： 
  + QQ技术1群：728661815（1群已满，请加4群）
  + QQ技术2群：117698356（2群已满，请加4群）
  + QQ技术3群：891252940（3群已满，请加4群）
  + QQ技术4群：985137094
* 微信:  robot4xiihoo
* 微信公众号: 小虎哥哥爱学习
* 邮箱:  robot4xiihoo@163.com
* 源码GitHub:  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
* 源码Gitee(访问更快):  https://gitee.com/xiihoo-robot/Books_Robot_SLAM_Navigation
* 淘宝:  https://xiihoo.taobao.com
* B站:   https://space.bilibili.com/66815220
* 购书链接：https://item.jd.com/13041503.html
## 资料汇总下载
* 百度网盘链接： https://pan.baidu.com/s/1nHbI0mi-iM72NAcQlAU1uQ?pwd=1234
* 提取码：1234
* 百度网盘资料目录预览：
  + 0-SLAM学习思维导图
  + 0-先导课
  + 1-第1季
  + 测试数据集
  + 工具软件
  + 课件
  + 微课
  + 自己动手做一台SLAM导航机器人
## 图书封面
![avatar](http://xiihoo.com/static/image/book_front_800x800.jpg)
## 目录
<ul>
  <li><font color=#008000 >序</font></li>
  <li><font color=#008000 >前言</font></li>
  <li><font color=#008000 >编程基础篇</font></li>
  <li>第1章 ROS入门必备知识 2</li>
  <li>第2章 C++编程范式 29</li>
  <li>第3章 OpenCV图像处理 35</li>
  <li><font color=#008000 >硬件基础篇</font></li>
  <li>第4章 机器人传感器 56</li>
  <li>第5章 机器人主机 119</li>
  <li>第6章 机器人底盘 132</li>
  <li><font color=#008000 >SLAM篇</font></li>
  <li>第7章 SLAM中的数学基础 158</li>
  <li>第8章 激光SLAM系统 223</li>
  <li>第9章 视觉SLAM系统 272</li>
  <li>第10章 其他SLAM系统 344</li>
  <li><font color=#008000 >自主导航篇</font></li>
  <li>第11章 自主导航中的数学基础 418</li>
  <li>第12章 典型自主导航系统 470</li>
  <li>第13章 机器人SLAM导航综合实战 502</li>
  <li>附录A Linux与SLAM性能优化的探讨 512</li>
  <li>附录B 习题 523</li>
</ul>

## 环境要求
* ubuntu 18.04 LTS
* ROS melodic

## 仓库内容说明
* **1.例程源码**
  + chapter_01: **ROS入门必备知识**
    - topic_example 话题通信
    - service_example 服务通信
    - action_example 动作通信
  + chapter_02: **C++编程范式**
    - g++_compile 利用g++编译
    - make_compile 利用make编译
    - cmake_compile 利用cmake编译
  + chapter_03: **OpenCV图像处理**
    - image_from_img 从图片文件中获取图像数据
    - image_from_vid 从视频文件中获取图像数据
    - image_from_cam 从相机设备中获取图像数据
    - calc_hist 直方图均衡
    - xfeatures2d_example 图像特征点提取
    - ikun_opencv_example 图像处理综合实战案例
  + chapter_04: **机器人传感器**
    - imu_tk IMU内参标定
    - imu_utils IMU内参标定
    - imu_tools Madgwick姿态融合 
    - ManhonyAHRS Manhony姿态融合
  + chapter_05: **机器人主机**
  + chapter_06: **机器人底盘**
  + chapter_07: **SLAM中的数学基础**
  + chapter_08: **激光SLAM系统**
    - gmapping gmapping激光SLAM系统
    - cartographer cartographer激光SLAM系统
    - loam LOAM激光SLAM系统
  + chapter_09: **视觉SLAM系统**
    - orb_slam2 ORB_SLAM2视觉SLAM系统
    - lsd_slam LSD_SLAM视觉SLAM系统
    - svo SVO视觉SLAM系统
  + chapter_10: **其他SLAM系统**
    - rtabmap rtabmap激光视觉融合SLAM系统
    - vins VINS视觉惯导融合SLAM系统
    - LeNet-5-tensorflow LeNet-5卷积神经网络
    - CNN_SLAM 基于CNN的SLAM系统
    - DeepVO 基于深度学习的端到端SLAM系统
  + chapter_11: **自主导航中的数学基础**
  + chapter_12: **典型自主导航系统**
    - ros-navigation 
    - riskrrt
    - autoware
  + chapter_13: **机器人SLAM导航综合实战**
    - patrol_fsm 多目标点巡逻
  + 更新中......
* **2.课件**
  + 0-先导课：课程大纲.pdf
  + 0-先导课：如何安装Ubuntu系统.pdf
  + 0-先导课：SLAM的应用价值与技术难点.pdf
  + 1-第1季：第0章_SLAM发展综述.pdf
  + 1-第1季：第1章_ROS入门必备知识.pdf
  + 1-第1季：第2章_C++编程范式.pdf
  + 1-第1季：第3章_OpenCV图像处理.pdf
  + 1-第1季：第4章_机器人传感器.pdf
  + 1-第1季：第5章_机器人主机.pdf
  + 1-第1季：第6章_机器人底盘.pdf
  + 1-第1季：第7章_SLAM中的数学基础.pdf
  + 1-第1季：第8章_激光SLAM系统.pdf
  + 1-第1季：第9章_视觉SLAM系统.pdf
  + 1-第1季：第10章_其他SLAM系统.pdf
  + 更新中......
  + **PPT课件下载说明:**
    - 由于PPT课件文件较大，放在github和gitee仓库会导致下载卡死或下载慢的问题，请前往百度网盘下载
    - 百度网盘链接： https://pan.baidu.com/s/1nHbI0mi-iM72NAcQlAU1uQ?pwd=1234
    - 提取码：1234
* **3.习题答案**
  + 附录B_习题.pdf
  + 更新中......
* **4.学习思维导图**
  + 学习思维导图.png
  + 学习思维导图.xmind
* **工具软件**
  + 三维空间可视化工具XH-3D-VIEW(V1.0.1).zip
* **自己动手做一台SLAM导航机器人**
  + code
  + 概述.pdf
  + 前言.pdf
  + 第1章：Linux基础.pdf
  + 第2章：ROS入门.pdf
  + 第3章：感知与大脑.pdf
  + 第4章：差分底盘设计.pdf
  + 第5章：树莓派3开发环境搭建.pdf
  + 第6章：SLAM建图与自主避障导航.pdf
  + 第7章：语音交互与自然语言处理.pdf
  + 附录A：用于ROS机器人交互的Android手机APP开发.pdf
  + 附录B：用于ROS机器人管理调度的后台服务器搭建.pdf
  + 附录C：如何选择ROS机器人平台进行SLAM导航入门.pdf
## 知乎专栏教程
* 知乎：https://www.zhihu.com/people/hiram_zhang
* 知乎专栏：
  + 《机器人SLAM导航核心技术与实战》笔记
    https://www.zhihu.com/column/c_1363309952996577280
  + 自己动手做一台SLAM导航机器人
    https://www.zhihu.com/column/c_1084087088789569536

## 视频教程
* B站：https://space.bilibili.com/66815220
* 视频列表：
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=1">【先导课】1.课程大纲-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=2">【先导课】1.1.课程大纲-学习思维导图（上）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=3">【先导课】1.2.课程大纲-学习思维导图（下）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=4">【先导课】1.3.课程大纲-全书内容速览-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=5">【先导课】1.4.下集预告-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=6">【先导课】2.如何安装Ubuntu系统-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=7">【先导课】2.1.如何安装Ubuntu系统-操作系统概念-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=8">【先导课】2.2.如何安装Ubuntu系统-Linux操作系统-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=9">【先导课】2.3.如何安装Ubuntu系统-Ubuntu发行版-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=10">【先导课】2.4.如何安装Ubuntu系统-物理机安装Ubuntu（独立硬盘）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=11">【先导课】2.5.如何安装Ubuntu系统-物理机安装Ubuntu（独立硬盘双系统）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=12">【先导课】2.6.如何安装Ubuntu系统-物理机安装Ubuntu（单硬盘双系统）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=13">【先导课】2.7.如何安装Ubuntu系统-虚拟机安装Ubuntu（VMware方法）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=14">【先导课】2.8.如何安装Ubuntu系统-虚拟机安装Ubuntu（virtualbox方法）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=15">【先导课】2.9.如何安装Ubuntu系统-Ubuntu系统基础入门-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=16">【先导课】3.SLAM的应用价值与技术难点-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=17">【先导课】3.1.SLAM的应用价值与技术难点-SLAM价值-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=18">【先导课】3.2.SLAM的应用价值与技术难点-产业应用与生态-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=19">【先导课】3.3.SLAM的应用价值与技术难点-核心技术与难点-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=20">【第1季】0.第0章_SLAM发展综述-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=21">【第1季】0.1.第0章_SLAM发展综述-梳理定位导航技术-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=22">【第1季】0.2.第0章_SLAM发展综述-揭秘SLAM技术路线-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=23">【第1季】0.3.第0章_SLAM发展综述-展望SLAM未来趋势-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=24">【第1季】1.第1章_ROS入门必备知识-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=25">【第1季】1.1.第1章_ROS入门必备知识-ROS简介-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=26">【第1季】1.2.第1章_ROS入门必备知识-ROS开发环境搭建-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=27">【第1季】1.3.第1章_ROS入门必备知识-ROS系统架构-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=28">【第1季】1.4.第1章_ROS入门必备知识-ROS调试工具-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=29">【第1季】1.5.第1章_ROS入门必备知识-ROS节点通信-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=30">【第1季】1.6.第1章_ROS入门必备知识-ROS其他重要概念-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=31">【第1季】1.7.第1章_ROS入门必备知识-ROS2.0展望-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=32">【第1季】2.第2章_C++编程范式-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=33">【第1季】2.1.第2章_C++编程范式-C++工程的组织结构-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=34">【第1季】2.2.第2章_C++编程范式-C++代码的编译方法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=35">【第1季】2.3.第2章_C++编程范式-C++编程风格指南-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=36">【第1季】3.第3章_OpenCV图像处理-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=37">【第1季】3.1.第3章_OpenCV图像处理_认识图像数据-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=38">【第1季】3.2.第3章_OpenCV图像处理_图像滤波-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=39">【第1季】3.3.第3章_OpenCV图像处理_图像变换-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=40">【第1季】3.4.第3章_OpenCV图像处理_图像特征点提取-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=41">【第1季】3.5.第3章_OpenCV图像处理_拓展-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=42">【第1季】4.第4章_机器人传感器-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=43">【第1季】4.1.第4章_机器人传感器_惯性测量单元-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=44">【第1季】4.2.第4章_机器人传感器_激光雷达-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=45">【第1季】4.3.第4章_机器人传感器_相机-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=46">【第1季】4.4.第4章_机器人传感器_带编码器的减速电机-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=47">【第1季】5.第5章_机器人主机-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=48">【第1季】5.1.第5章_机器人主机_X86与ARM主机对比-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=49">【第1季】5.2.第5章_机器人主机_ARM主机树莓派3B+-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=50">【第1季】5.3.第5章_机器人主机_ARM主机RK3399-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=51">【第1季】5.4.第5章_机器人主机_ARM主机Jetson-tx2-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=52">【第1季】5.5.第5章_机器人主机_分布式架构主机-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=53">【第1季】6.第6章_机器人底盘-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=54">【第1季】6.1.第6章_机器人底盘_底盘运动学模型-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=55">【第1季】6.2.第6章_机器人底盘_底盘性能指标-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=56">【第1季】6.3.第6章_机器人底盘_典型机器人底盘搭建-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=57">【第1季】7.第7章_SLAM中的数学基础-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=58">【第1季】7.1.第7章_SLAM中的数学基础_SLAM发展简史-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=59">【第1季】7.2.第7章_SLAM中的数学基础_SLAM中的概率理论-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=60">【第1季】7.3.第7章_SLAM中的数学基础_估计理论-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=61">【第1季】7.4.第7章_SLAM中的数学基础_基于贝叶斯网络的状态估计-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=62">【第1季】7.5.第7章_SLAM中的数学基础_基于因子图的状态估计-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=63">【第1季】7.6.第7章_SLAM中的数学基础_典型SLAM算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=64">【第1季】7.7.第7章_SLAM中的数学基础_SFM、BA和SLAM比较-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=65">【第1季】8.第8章_激光SLAM系统-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=66">【第1季】8.1.第8章_激光SLAM系统_Gmapping算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=67">【第1季】8.2.第8章_激光SLAM系统_Cartographer算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=68">【第1季】8.3.第8章_激光SLAM系统_LOAM算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=69">【第1季】9.第9章_视觉SLAM系统-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=70">【第1季】9.1.第9章_视觉SLAM系统_ORB-SLAM2算法（上）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=71">【第1季】9.1.第9章_视觉SLAM系统_ORB-SLAM2算法（下）-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=72">【第1季】9.2.第9章_视觉SLAM系统_LSD-SLAM算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=73">【第1季】9.3.第9章_视觉SLAM系统_SVO算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=74">【第1季】10.第10章_其他SLAM系统-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=75">【第1季】10.1.第10章_其他SLAM系统_RTABMAP算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=76">【第1季】10.2.第10章_其他SLAM系统_VINS算法-视频讲解</a>
  + <a href="https://www.bilibili.com/video/BV1jS4y1a7Lz?p=77">【第1季】10.3.第10章_其他SLAM系统_机器学习与SLAM-视频讲解</a>
  + 更多精彩内容，正在更新...
## 第三方库引用说明
本项目最终是为了将书本中学习到的理论知识应用到实战，并通过动手实践制作出一台可以实际运行的SLAM导航机器人（也就是“xiihoo”机器人）。实战过程中，除了需要搭建机器人硬件，还需要搭建机器人软件及相应的软件运行环境。系统和软件版本并不是越新越好，够用就行了，大家切记。本项目推荐使用ubuntu18.04操作系统和ROS melodic版本，除此之外还涉及到众多第三方库环境，为了方便大家学习将其整理如下：
* 操作系统版本：ubuntu18.04
  + https://ubuntu.com/
  + https://releases.ubuntu.com/
* ROS平台版本：ROS melodic
    + https://wiki.ros.org/melodic
    + https://github.com/ros
    + https://github.com/ros/rosdistro/tree/master/melodic
    + https://github.com/ros/ros_comm/tree/melodic-devel
    + https://github.com/ros/ros/tree/melodic-devel
    + https://index.ros.org/packages/#melodic
* OpenCV库版本：opencv-3.2.0 opencv_contrib-3.2.0
  + https://github.com/opencv/opencv/tree/3.2.0
  + https://github.com/opencv/opencv_contrib/tree/3.2.0
* Gmapping版本：slam_gmapping-ros-melodic openslam_gmapping-ros-melodic
  + https://github.com/ros-perception/slam_gmapping/tree/melodic-devel
  + https://github.com/ros-perception/openslam_gmapping/tree/melodic-devel
* Cartographer版本：cartographer_ros-1.0.0 cartographer-1.0.0 ceres-solver-1.13.0
  + https://github.com/cartographer-project/cartographer_ros/releases/tag/1.0.0
  + https://github.com/cartographer-project/cartographer/releases/tag/1.0.0
  + https://github.com/ceres-solver/ceres-solver/releases/tag/1.13.0
* LOAM版本：loam_velodyne
  + https://github.com/laboshinl/loam_velodyne
  + https://github.com/HKUST-Aerial-Robotics/A-LOAM
  + https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
* ORB-SLAM版本：ORB_SLAM2
  + https://github.com/raulmur/ORB_SLAM2
  + https://gitlab.com/libeigen/eigen/-/tree/3.2.10
  + https://github.com/stevenlovegrove/Pangolin
  + https://github.com/dorian3d/DBoW2
  + https://github.com/RainerKuemmerle/g2o
* 三维空间运动几何库：Sophus manif
  + https://github.com/strasdat/Sophus
  + https://github.com/artivis/manif
* 点云库：PCL
  + https://github.com/PointCloudLibrary/pcl
* PnP库：EPnP
  + https://github.com/cvlab-epfl/EPnP
* LSD-SLAM版本：
  + https://github.com/tum-vision/lsd_slam
* SVO版本：
  + https://github.com/uzh-rpg/rpg_svo
* RTABMAP版本：
  + https://github.com/introlab/rtabmap_ros
  + https://github.com/introlab/rtabmap
  + https://github.com/RainerKuemmerle/g2o
  + https://github.com/borglab/gtsam
  + https://github.com/ethz-asl/libpointmatcher
* IMU外参标定库：
  + https://github.com/ethz-asl/kalibr
* IMU融合：
  + https://github.com/ethz-asl/ethzasl_sensor_fusion
  + https://github.com/ethz-asl/ethzasl_msf
* VINS版本：
  + https://github.com/HKUST-Aerial-Robotics/VINS-Mono
* 机器学习Python例程库：
  + https://github.com/the-learning-machine/ML-algorithms-python
* CNN_SLAM版本：
  + https://github.com/iitmcvg/CNN_SLAM
* DeepVO版本：
  + https://github.com/themightyoarfish/deepVO
* 导航框架版本：navigation-ros-melodic
  + https://github.com/ros-planning/navigation/tree/melodic-devel
* 里程计融合：
  + https://github.com/ros-planning/robot_pose_ekf
* 激光里程计：
  + https://github.com/MAPIRlab/rf2o_laser_odometry
* 路径规划插件：
  + https://github.com/srl-freiburg/srl_global_planner
  + https://github.com/rst-tu-dortmund/teb_local_planner
* 环境探索：
  + https://github.com/paulbovbel/frontier_exploration
  + https://github.com/hasauino/rrt_exploration
* riskrrt导航框架：
  + https://github.com/spalanza/riskrrt_ros
* autoware导航框架：
  + https://github.com/Autoware-AI/autoware.ai
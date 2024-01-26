/******************************************************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
******************************************************************/

#include<stdio.h>   
#include<stdlib.h>   
#include<errno.h>   
#include<string.h>   
#include<sys/types.h>   
#include<netinet/in.h>   
#include<sys/socket.h>   
#include<sys/wait.h>   
#include<sys/stat.h>   
#include<fcntl.h>   
#include<unistd.h>   
#include <arpa/inet.h>   
#include<netdb.h>   
  
#define PORT 7773   
#define MAXDATASIZE 256   
  
int main(int argc,char *argv[])  
{  
  /******************************************************************
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
  *******************************************************************/
    int socket_fd;  
    struct sockaddr_in my_addr,user_addr;  
    char buf[MAXDATASIZE];  
    int so_broadcast=1;  
    socklen_t size;  
    char my_ip[12];  
  
    my_addr.sin_family=AF_INET;  
    my_addr.sin_port=htons(PORT);  
    my_addr.sin_addr.s_addr=inet_addr("255.255.255.255");  
    bzero(&(my_addr.sin_zero),8);  
     
    user_addr.sin_family=AF_INET;  
    user_addr.sin_port=htons(PORT);  
    user_addr.sin_addr.s_addr=htonl(INADDR_ANY);  
    bzero(&(user_addr.sin_zero),8);  
    if((socket_fd=(socket(AF_INET,SOCK_DGRAM,0)))==-1) {  
        perror("socket");  
        exit(1);  
    }  
    setsockopt(socket_fd,SOL_SOCKET,SO_BROADCAST,&so_broadcast,sizeof(so_broadcast));  
    if((bind(socket_fd,(struct sockaddr *)&user_addr,  
                        sizeof(struct sockaddr)))==-1) {  
        perror("bind");  
        exit(1);  
    }  


    while(1) 
    {  
        strcpy(buf,inet_ntoa(user_addr.sin_addr));  
    	sendto(socket_fd,buf,strlen(buf),0,(struct sockaddr *)&my_addr,sizeof(my_addr));  
	    size=sizeof(user_addr);  
    	recvfrom(socket_fd,buf,MAXDATASIZE,0,(struct sockaddr *)&user_addr,&size);  
    	strcpy(my_ip,inet_ntoa(user_addr.sin_addr));  
    	printf("my_ip:%s\n",inet_ntoa(user_addr.sin_addr));  
        sleep(1);  
       
    } 
    return 0;  
}  

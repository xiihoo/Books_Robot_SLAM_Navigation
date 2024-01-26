/************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
**************************/

/*
 *  tuling_arv.cpp
 *  tuling_arv_node
*/
 
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>
#include<sstream>
#include<jsoncpp/json/json.h>
#include<curl/curl.h>
#include<string>
#include<exception>
 
using namespace std;
 
int flag = 0;
string result;
 
 int writer(char *data, size_t size, size_t nmemb, string *writerData)
{
    unsigned long sizes = size * nmemb;
    if (writerData == NULL)
        return -1;
 
    writerData->append(data, sizes);
 
    return sizes;
}
 
/*
*   解析图灵服务器返回的Json string
*/
int parseJsonResonse(string input)
{
   Json::Value root;
   Json::Reader reader;
   bool parsingSuccessful = reader.parse(input, root);
   if(!parsingSuccessful)
   {
       std::cout<<"!!! Failed to parse the response data"<< std::endl;
        return -1;
   }
   const Json::Value code = root["code"];
   const Json::Value text = root["text"];
   result = text.asString();
   flag = 1;
 
   std::cout<< "response code:" << code << std::endl;
   std::cout<< "response text:" << result << std::endl;
 
   return 0;
}
 
/*
*   将input字腹发送到图灵服务器接受json string
*/
int HttpPostRequest(string input)
{
    string buffer;
 
    std::string strJson = "{";
    strJson += "\"key\" : \"818df27408a34ee18e8db525f3ecc147\","; //双引号前加／防转仪
    strJson += "\"info\" : ";
    strJson += "\"";
    strJson += input;
    strJson += "\"";
    strJson += "}";
 
    std::cout<<"post json string: " << strJson << std::endl;
 
     try
    {
        CURL *pCurl = NULL;
        CURLcode res;
        // In windows, this will init the winsock stuff
        curl_global_init(CURL_GLOBAL_ALL);
 
        // get a curl handle
        pCurl = curl_easy_init();
        if (NULL != pCurl)
        {
            // 设置超时时间为10秒
            curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 10);
 
            // First set the URL that is about to receive our POST.
            // This URL can just as well be a
            // https:// URL if that is what should receive the data.
            curl_easy_setopt(pCurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");
            //curl_easy_setopt(pCurl, CURLOPT_URL, "http://192.168.0.2/posttest.cgi");
 
            // 设置http发送的内容类型为JSON
            curl_slist *plist = curl_slist_append(NULL,
                "Content-Type:application/json;charset=UTF-8");
            curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);
 
            // 设置要POST的JSON数据
            curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, strJson.c_str());
 
            curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, writer);
 
            curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);
 
            // Perform the request, res will get the return code
            res = curl_easy_perform(pCurl);
            // Check for errors
            if (res != CURLE_OK)
            {
                printf("curl_easy_perform() failed:%s\n", curl_easy_strerror(res));
            }
            // always cleanup
            curl_easy_cleanup(pCurl);
        }
        curl_global_cleanup();
    }
    catch (std::exception &ex)
    {
        printf("curl exception %s.\n", ex.what());
    }
    if(buffer.empty())
    {
      std::cout<< "!!! ERROR The Tuling sever response NULL" << std::endl;
    }
    else
    {
        parseJsonResonse(buffer);
    }
 
    return 0;
 
}
 
/*
*   当voice/tuling_arv_topic话题有消息时，调用HttpPostRequest向图灵服务器发送内容，返回结果。
*/
void arvCallBack(const std_msgs::String::ConstPtr &msg)
{
    std::cout<<"your quesion is: " << msg->data << std::endl;
	
	
	//-------------tuling---------------------
    HttpPostRequest(msg->data);
	
	//-------------other----------------------
	//......
}
 
int main(int argc, char **argv)
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
    using namespace std;
    ros::init(argc, argv,"i_qa_nlp");
    ros::NodeHandle nd;
 
 
	string iatTopic="/voice/iat_Pub";
	ros::param::get("~iatTopic",iatTopic);
	ROS_INFO("get param,iatTopic:%s",iatTopic.c_str());
    ros::Subscriber sub = nd.subscribe(iatTopic.c_str(), 10, arvCallBack);
	
	string ttsTopic="/xfwords";
	ros::param::get("~ttsTopic",ttsTopic);
	ROS_INFO("get param,ttsTopic:%s",ttsTopic.c_str());
    ros::Publisher pub = nd.advertise<std_msgs::String>(ttsTopic.c_str(), 10);
    ros::Rate loop_rate(10);
 
    while(ros::ok())
    {
        if(flag)
        {
            std_msgs::String msg;
            msg.data = result;
            pub.publish(msg);
            flag = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
 
 
}

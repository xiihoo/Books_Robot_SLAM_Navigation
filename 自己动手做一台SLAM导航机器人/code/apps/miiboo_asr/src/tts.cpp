/************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
**************************/

/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>


#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(15*1000); //防止频繁占用CPU
	}//合成状态synth_status取值请参阅《讯飞语音云API文档》
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

string aplay_addr="plughw:0,0";
void xfcallback(const std_msgs::String::ConstPtr& msg)
{
  char cmd[2000];
  const char* text;
  int         ret                  = MSP_SUCCESS;
  const char* session_begin_params = "voice_name = nannan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 0";
  const char* filename             = "tts_sample.wav"; //合成的语音文件名称
  std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;
  text = msg->data.c_str(); 
  /* 文本合成 */
  printf("开始合成 ...\n");
  ret = text_to_speech(text, filename, session_begin_params);
  if (MSP_SUCCESS != ret)
  {
       printf("text_to_speech failed, error code: %d.\n", ret);
  }
  printf("合成完毕\n");
  ros::NodeHandle n; 
  n.setParam("my_param_state", "speech_start");

  unlink("/tmp/cmd");  
  mkfifo("/tmp/cmd", 0777);  
  //system("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'");
  //system("aplay -D plughw:1,0 ./tts_sample.wav");
  //system("aplay -D plughw:1,0 ./tts_sample.wav");
  char str_cmd[128];
  memset(str_cmd,0,sizeof(str_cmd));
  strcat (str_cmd,"aplay ./tts_sample.wav  -D ");
  strcat (str_cmd,aplay_addr.c_str());
  printf("str_cmd:%s\n", str_cmd);
  system(str_cmd);
  
  usleep(500*1000);
  printf("Mplayer Run Success\n");
  n.setParam("my_param_state", "speech_stop");
  //ROS_ERROR("setParam my_param_state speech_stop");

}




void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}

int main(int argc, char* argv[])
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
	int         ret                  = MSP_SUCCESS;
	//const char* login_params         = "appid = 58249817, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	const char* login_params         = "appid = 56ee43d0, work_dir = /tmp/";//登录参数,appid与msc库绑定,请勿随意改动
   
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/

	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		/*goto exit ;*///登录失败，退出登录
        toExit();
	}

    ros::init(argc,argv,"i_xf_tts");
    ros::NodeHandle n;
	n.setParam("my_param_state", "speech_stop");
	
	string ttsTopic="/xfwords";
	ros::param::get("~ttsTopic",ttsTopic);
	ROS_INFO("get param,ttsTopic:%s",ttsTopic.c_str());
	
	string my_aplay_addr="";
	ros::param::get("~aplay_addr",my_aplay_addr);
	aplay_addr=my_aplay_addr;
	ROS_INFO("get param,my_aplay_addr:%s",my_aplay_addr.c_str());
	
    ros::Subscriber sub =n.subscribe(ttsTopic.c_str(),1,xfcallback);
	
	//-----------tmp speek----------------
	ros::Publisher pub = n.advertise<std_msgs::String>(ttsTopic.c_str(), 10);
	std_msgs::String msg;
	//msg.data = voice.voice_recongnition_text+6;
	//msg.data = "你好，欢迎使用智能小车语音控制系统";
	msg.data = "你好，欢迎使用miiboo机器人语音交互系统";
	pub.publish(msg);
	//-----------------------------------
	ROS_INFO("=======================");
    ros::spin();



exit:
	printf("按任意键退出 ...\n");
	getchar();
	MSPLogout(); //退出登录

	return 0;
}


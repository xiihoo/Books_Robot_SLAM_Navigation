/************************
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
**************************/

/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/
#include "voice/voice.h"
#include <alsa/asoundlib.h>
static Voice voice;
int size;
snd_pcm_t* handle; //PCI设备句柄
snd_pcm_hw_params_t* pcm_params;//硬件信息和PCM流配置
snd_pcm_uframes_t frames;
char *once_upload_pcm_buffer = NULL; //该音频缓冲区一旦满足len（下方）就上传科大讯飞
char *rec_pcm_buffer; 
const char* session_id  = NULL;
int check_state();
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

int flag_WakeUp    = 0 ;
int flag_ok = 0 ;
int flag_no = 0 ;

/*科大讯飞：上传声音关键字 */
/* Upload User words */
static int upload_userwords()
{
	char*			userwords	=	NULL;
	size_t			len			=	0;
	size_t			read_len	=	0;
	FILE*			fp			=	NULL;
	int				ret			=	-1;

	fp = fopen("/home/robot/catkin_ws/src/xf-ros-master/xfei_asr/src/userwords.txt", "rb");
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit;
	}

	fseek(fp, 0, SEEK_END);
	len = ftell(fp); //获取音频文件大小
	fseek(fp, 0, SEEK_SET);  					
	
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit;
	}

	read_len = fread((void*)userwords, 1, len, fp); //读取用户词表内容
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit;
	}
	userwords[len] = '\0';
	
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //上传用户词表
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit;
	}
	
upload_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);
		userwords = NULL;
	}
	
	return ret;
}


static void show_result(char *string, char is_over)
{
    flag_ok=1;	
    printf("\rResult:[ %s ]",  string);
    if(is_over)
		putchar('\n');

}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}


/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
        flag_no=1;
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(i++ < 15)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
        flag_no=1;
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */

void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");
   // printf("%s", *msg->data.c_str());
    usleep(700*1000);
    flag_WakeUp=1;
}







//--------------------------------------------------------
static void show_sys_info(char *str)
{
	printf("%s", str);
	/*
	 * 这个地方添加写入日志文件的操作！
	 */
}


/* 合成的wav声音文件：默认wav音频头部数据 */
static wave_pcm_hdr default_wav_hdr = 
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
/*
 * 功能：设置voice的工作模式
 */
int set_voice_mode(int mode)
{
	if (mode == NETWORK_CONNECTED)
    {
				if (voice.voice_main_switch == SWITCH_ON) //如果当前已经开启语音功能，即登陆过科大讯飞
				{
					return 0;	
				}
                int ret = MSP_SUCCESS;
                //登陆到科大讯飞，登陆参数
                char login_params[100];
                memset(login_params, 0, sizeof(login_params));
                strcat(login_params, "appid = ");
                strcat(login_params, voice.xfyun_appid);
                strcat(login_params, ", work_dir = .");
		
                /* 用户登录 */
                ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数    
                if (MSP_SUCCESS != ret)
                {
                        printf("MSPLogin failed (科大讯飞账号登陆失败！) , Error code %d.\n", ret);
                        show_sys_info("voice初始化：语音识别初始化失败！\n");
                        MSPLogout();                                      //退出登录
                        return -1;
                }

                int upload_on = 0;       //是否上传用户词表，默认为关掉上传用户此表
                if (upload_on)
                {
                        show_sys_info("voice_recongnition: 上传用户词表 ...\n");
                        ret = upload_userwords();
                        if (MSP_SUCCESS != ret)
                        {
                                MSPLogout();                                      //退出登录
                                return -1;
                        }
                        show_sys_info("voice_recongnition: 上传用户词表成功\n");
                }
                voice.voice_main_switch = SWITCH_ON;
				return 0;
        }
        else if (mode ==  NETWORK_DISCONNECTED)
        {
				voice.voice_main_switch = SWITCH_OFF;
                show_sys_info("不支持语音聊天！\n");
				return 0;
        }		
		else
		{
			show_sys_info("voice设置启动模式：出错！\n");
			return -1;
		}
}

/*
 *功能：初始化语音模块
 *参数：mode:联网模式； xfyun_appid：科大讯飞开发包的APPID
 */
void voice_init(int mode, const char *xfyun_appid, const char *usb_audio_addr)
{
	voice.voice_main_switch = SWITCH_OFF;
	voice.recongnition_switch = SWITCH_OFF;
	voice.sound_box_ongoing_flag = 0;//允许录音,不需要唤醒词

	pthread_mutex_init(&(voice.mutex_voice_params), NULL);  //默认属性初始化声音参数内存空间的线程锁
	memset(voice.voice_recongnition_text, 0, sizeof(voice.voice_recongnition_text));
	memset(voice.smart_reply_text, 0, sizeof(voice.smart_reply_text));
	memset(voice.smart_reply_code, 0, sizeof(voice.smart_reply_code));
	memset(voice.xfyun_appid, 0, sizeof(voice.xfyun_appid));
	memset(voice.usb_audio_addr, 0, sizeof(voice.usb_audio_addr));

	//科大讯飞
	if (xfyun_appid == NULL || strlen(xfyun_appid) <= 0)
	{
		show_sys_info("voice初始化错误：讯飞APPID为空，语音聊天控制功能将有限制！\n");
	}
	else
	{
		strcpy(voice.xfyun_appid, xfyun_appid);	
	}
	
	
	//USB声卡
	if (usb_audio_addr == NULL || strlen(usb_audio_addr) <= 0)
	{
		show_sys_info("voice初始化错误：USB声卡地址信息为空，语音聊天控制功能将有限制！\n");
		
	}
	else
	{
		strcpy(voice.usb_audio_addr, usb_audio_addr);
	}

	//设置voice模式
	if (set_voice_mode(mode) < 0)
	{
		show_sys_info("voice初始化失败：设置启动模式错误！\n");
		return ;
	}
	
	show_sys_info("voice总初始化：成功！\n");
}







/*
 * 功能：读取麦克风数据并识别出文字
 * 参数：rec_result：识别出的文字存放位置
 *       rec_result_size：存放文字空间大小
 * 返回值：成功返回0，失败返回-1
 * 说明：当在一定的时间（检测次数，默认为15），检测不到声音时（音量的阀值valume_threshold默认为6），则进行识别并退出！
 *       当检测到声音时（音量的阀值valume_threshold默认为6），之后如果连续的音量大小小于一定时间（检测次数，默认为3次），则进行识别并退出！
 *       如果需要调节音量阀值，请自己修改函数内部的数值
 */

static int speech_recognition(char *rec_result, int rec_result_size)
{
		/******************录制声音的各种参数部分**********************/	
		int rc;
		printf("speech_recognition...\n");
        int ret = -1;
        unsigned int val;
        int dir=0;
		int channels = default_wav_hdr.channels;
        int frequency = default_wav_hdr.samples_per_sec;
        int bit = default_wav_hdr.bits_per_sample;
        int datablock = default_wav_hdr.block_align;
		int final_return = -1;  //程序最终返回值，放到这里
		/* Open PCM device for recording (capture). */
    	printf("正在打开录音设备...\n");
    	rc = snd_pcm_open(&handle, voice.usb_audio_addr, SND_PCM_STREAM_CAPTURE, 0);
    	if (rc < 0)
    	{
        	fprintf(stderr,  "unable to open pcm device: %s/n",  snd_strerror(rc));
			sleep(1);	 //可能出现频繁调用该函数，所以休眠一下
        	return -1;
    	}
    	printf("打开完毕！请对我说话...\n");
		snd_pcm_hw_params_alloca(&pcm_params); //分配params结构体
        if(rc<0)
        {
                perror("\nsnd_pcm_hw_params_alloca:");
				snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1); 
                return -1;
        }
        rc=snd_pcm_hw_params_any(handle, pcm_params);//初始化params
        if(rc<0)
        {
                perror("\nsnd_pcm_hw_params_any:");
				snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1); 
                return -1;
        }
        rc=snd_pcm_hw_params_set_access(handle, pcm_params, SND_PCM_ACCESS_RW_INTERLEAVED); //初始化访问权限
        if(rc<0)
        {
                perror("\nsed_pcm_hw_set_access:");
                snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1);
				return -1;
        }	
		//采样位数
        switch(bit/8)
        {
				case 1:rc=snd_pcm_hw_params_set_format(handle, pcm_params, SND_PCM_FORMAT_U8);
						break ;
				case 2:rc=snd_pcm_hw_params_set_format(handle, pcm_params, SND_PCM_FORMAT_S16_LE);
						break ;
				case 3:rc=snd_pcm_hw_params_set_format(handle, pcm_params, SND_PCM_FORMAT_S24_LE);
						break ;
				default:
				rc = -1;
        }
		if (rc<0)
		{
				perror("\nsnd_pcm_hw_params_set_format:");
				snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1);
				return -1;
		}
		rc=snd_pcm_hw_params_set_channels(handle, pcm_params, channels); //设置声道,1表示单声>道，2表示立体声
        if(rc<0)
        {
                perror("\nsnd_pcm_hw_params_set_channels:");
                snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1);
				return -1;
        }

        val = frequency;
        rc=snd_pcm_hw_params_set_rate_near(handle, pcm_params, &val, &dir); //设置>频率
        if(rc<0)
        {
                perror("\nsnd_pcm_hw_params_set_rate_near:");
                snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1);
				return -1;
        }
        rc = snd_pcm_hw_params(handle, pcm_params);   //这个函数有延时！大概1s左右，原因不知道！
        if(rc<0)
        {
       		perror("\nsnd_pcm_hw_params: ");
        	snd_pcm_drain(handle);
        	snd_pcm_close(handle);
			sleep(1);
			return -1;
        }	
		rc=snd_pcm_hw_params_get_period_size(pcm_params, &frames, &dir); /*获取周期长度*/
        if(rc<0)
        {
                perror("\nsnd_pcm_hw_params_get_period_size:");
                snd_pcm_drain(handle);
				snd_pcm_close(handle);
				sleep(1);
				return -1;
        }
		size = frames * datablock; /*4 代表数据快长度*/
	return final_return;
}




static int speech_recognition_while(char *rec_result, int rec_result_size)
{	
	int rc;
	int ret = -1;
    int final_return = -1;  //程序最终返回值，放到这里
	int first_get_voice=0; 
    first_get_voice=0; 
	
	/****************语音识别初始化部分*********************/
	const char* session_begin_params = "sub = iat, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8";	
	int errcode = MSP_SUCCESS;
	session_id = QISRSessionBegin(NULL, session_begin_params, &errcode); //听写不需要语法，第一个参数为NULL
	if (MSP_SUCCESS != errcode)
	{
			printf("\nQISRSessionBegin failed! error code:%d\n", errcode);
			//final_return = -1;
			return -2;
	}
	unsigned int total_len = 0;
	int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE; //音频状态
	int ep_stat = MSP_EP_LOOKING_FOR_SPEECH; //端点检测
	int rec_stat = MSP_REC_STATUS_SUCCESS;  //识别状态
	memset(rec_result, 0, rec_result_size);
        //memset(hints, 0, sizeof(hints));
	unsigned int upload_buffer_size = 0;   //此值满足len就上传
	rec_pcm_buffer = (char *)malloc(size); //此时size大小应该为1024
	int first = 1;
        unsigned int len = 10 * FRAME_LEN; //每次写入200ms音频(16k，16bit)：1帧音频20ms，10帧=200ms。16k采样率的16位音频，一帧的大小为640Byte
	int after_voice_zero_num = 0;          //检测到声音之后，音量连续为无的次数
	int after_voice_counter_ongoing = 0;   //检测到声音之后的音量为无计数器
	int front_voice_zero_num = 0;          //检测到声音之前，音量连续为无的次数
	int front_voice_counter_ongoing = 1;   //检测到声音之前的音量为无计数器，默认开启
	int valume_threshold = 3;              //声音阀值，小于等于该阀值的声音将视为无声音
	//char *once_upload_pcm_buffer = NULL; //该音频缓冲区一旦满足len（下方）就上传科大讯飞
    while (voice.recongnition_switch == SWITCH_ON)
	{
        int ret = 0;
		//录制一次音频！大小为size(1024)
		memset(rec_pcm_buffer, 0, size);
		rc = snd_pcm_readi(handle, rec_pcm_buffer, frames);  //frames大小应该为512， size大小为1024
		if (rc == -EPIPE)
		{
				fprintf(stderr, "overrun occurred/n");
				snd_pcm_prepare(handle);
		}
		else if (rc < 0)
		{
				fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
		}
		else if (rc != (int)frames)
		{
				fprintf(stderr, "short read, read %d frames/n", rc);
		}
		char *temp_buffer = (char *)malloc(upload_buffer_size + size);
		memset(temp_buffer, 0, upload_buffer_size + size);
		memcpy(temp_buffer, once_upload_pcm_buffer, upload_buffer_size); //原先的拷贝过去
		memcpy(temp_buffer+upload_buffer_size, rec_pcm_buffer, size);   //再添加新的
		if (first)
		{
			first = 0;
			aud_stat = MSP_AUDIO_SAMPLE_FIRST;
		}
		else
		{
			free(once_upload_pcm_buffer);
			once_upload_pcm_buffer = NULL;
			aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
		}
		once_upload_pcm_buffer = temp_buffer;
		upload_buffer_size += size;
		//如果当前录制的音频达到科大讯飞要求的大小，则上传科大讯飞
		if (upload_buffer_size >= len)
		{
			ret = QISRAudioWrite(session_id, (const void*)once_upload_pcm_buffer, upload_buffer_size, aud_stat, &ep_stat, &rec_stat);	
			if (MSP_SUCCESS != ret)
			{
				printf("\nQISRAudioWrite failed! error code:%d\n", ret);
                return -2;  
			}
			
			if (MSP_REC_STATUS_SUCCESS == rec_stat) //已经有部分听写结果
			{
					const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
					if (MSP_SUCCESS != errcode)
					{
							printf("\nQISRGetResult failed! error code: %d\n", errcode);
							return -2;  
					}
					if (NULL != rslt)
					{
							unsigned int rslt_len = strlen(rslt);
							total_len += rslt_len;
							if (total_len >= BUFFER_SIZE)
							{
									printf("\nno enough buffer for rec_result !\n");
									return -2; 
							}
							strncat(rec_result, rslt, rslt_len);
							break;
					}
			}

			//获取音量大小
			const char * para_name = "volume";
			char para_value[33] = {'\0'};
			unsigned int value_len = 33;
			ret = QISRGetParam(session_id, para_name, para_value, &value_len);
			if( MSP_SUCCESS != ret )
			{
					printf( "QISRGetParam failed, error code is: %d\n", ret );
					return -2;
			}
			int valume = 0;
        		valume = atoi(para_value);
			if(valume>0){	
				for(int ix=0;ix<valume;ix++)
				{
					printf("-", valume);
				}
				printf("\n", valume);
			}
			if(first_get_voice<5)
			{
				first_get_voice++; 
				
				check_state();
				voice.voice_type=voice.sound_box_ongoing_flag;
			}
			if (valume > valume_threshold) //检测到声音了
        	{
				if (!after_voice_counter_ongoing)   //如果没有开启检测到声音后的计数器开启，
				{
					after_voice_counter_ongoing = 1; //开启声音后计数器
					front_voice_counter_ongoing = 0; //关闭声音前计数器
				}
				after_voice_zero_num = 0;		 //每一次检测到有声音后都清零声音后计数器
        	}
			//感觉上面的获取音量的那一块放到上面最好，，等下再改看看		
			if (valume <= valume_threshold && after_voice_counter_ongoing == 1)
			{	
				if (after_voice_zero_num >= 2)
				{
					after_voice_counter_ongoing = 0;
					after_voice_zero_num = 0;
					front_voice_counter_ongoing = 1;
					front_voice_zero_num = 0;
					break;		
				}
				else
					after_voice_zero_num++;
			}
			if (valume <= valume_threshold && front_voice_counter_ongoing == 1)
			{
				if (front_voice_zero_num >= 1500)
				{
					after_voice_counter_ongoing = 0;
                                        after_voice_zero_num = 0;
                                        front_voice_counter_ongoing = 1;
                                        front_voice_zero_num = 0;
					break;			
				}
				else
					front_voice_zero_num++;
			}
			upload_buffer_size = 0; //使重新开始录制音频		
		}
    }

	errcode = QISRAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_stat, &rec_stat);
	if (MSP_SUCCESS != errcode)
	{
			printf("\nQISRAudioWrite failed! error code:%d \n", errcode);
			return -2;
	}

	while (MSP_REC_STATUS_COMPLETE != rec_stat)
	{
			const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
			if (MSP_SUCCESS != errcode)
			{
					printf("\nQISRGetResult failed, error code: %d\n", errcode);
					return -2;
			}
			if (NULL != rslt)
			{
					unsigned int rslt_len = strlen(rslt);
					total_len += rslt_len;
					if (total_len >= BUFFER_SIZE)
					{
							printf("\nno enough buffer for rec_result !\n");
							return -2;
					}
					strncat(rec_result, rslt, rslt_len);
					printf("音频转文字：%s voice.voice_type=%d\n", rec_result,voice.voice_type);
			}
			usleep(150*1000); //防止频繁占用CPU
	}
	final_return = 0;	
	QISRSessionEnd(session_id, "end");
}

int check_state()
{
	std::string state;
	ros::NodeHandle n;
	if (n.getParam("my_param_state", state))
	{
		if(strstr(state.c_str(),"speech_start")!=NULL)
		{
			voice.sound_box_ongoing_flag=1; //允许录音,需要唤醒词
		}
		else if(strstr(state.c_str(),"speech_stop")!=NULL)
		{
			voice.sound_box_ongoing_flag=0; //允许录音，不需要唤醒词
		}
	}
	else
	{
		return -1;
	}
	return 0;
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
    using namespace std;
	//1、ROS初始化
    ros::init(argc, argv, "i_xfiat");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sbu = n.subscribe("xfwakeup", 1000, WakeUp);
	
	string iatTopic="/voice/iat_Pub";
	ros::param::get("~iatTopic",iatTopic);
	ROS_INFO("get param,iatTopic:%s",iatTopic.c_str());
	ros::Publisher pub3 = n.advertise<std_msgs::String>(iatTopic.c_str(), 1000);
	
	string my_usb_audio_addr="plughw:1,0";
	ros::param::get("~usb_audio_addr",my_usb_audio_addr);
	ROS_INFO("get param,my_usb_audio_addr:%s",my_usb_audio_addr.c_str());



	int ret_speech_recognition=0;
continue_error:	
	//参数：      网络连接标志   科大讯飞appid      声卡地址
	//voice_init(NETWORK_CONNECTED, "56ee43d0", "plughw:CARD=U0x46d0x825");
	//voice_init(NETWORK_CONNECTED, "56ee43d0", "plughw:1,0");
	voice_init(NETWORK_CONNECTED, "56ee43d0", my_usb_audio_addr.c_str());
	voice.recongnition_switch = SWITCH_ON;
	show_sys_info("语音识别已经开启...\n");

	//1、音频转文字初始化
	memset(voice.voice_recongnition_text, 0, sizeof(voice.voice_recongnition_text));	
	ret_speech_recognition=speech_recognition(voice.voice_recongnition_text, sizeof(voice.voice_recongnition_text));
	if(ret_speech_recognition==-2)
	{
			//iat_exit:
			if (once_upload_pcm_buffer != NULL)
			{
				free(once_upload_pcm_buffer);
			}
			free(rec_pcm_buffer);	
			snd_pcm_drain(handle);
			snd_pcm_close(handle);
			QISRSessionEnd(session_id, "end");
			MSPLogout(); //退出登录科大讯飞
			show_sys_info("音频转文字初始化失败...goto continue_error\n");
			usleep(1000*1000);
			goto continue_error;
	}
	
    while(ros::ok())
    {
	
		int ret=check_state();
		if(ret==-1)
		{
			ROS_ERROR("===Failed to get param 'my_param_state'...continue");
			usleep(1000*1000);
			continue;
		}
		if (voice.recongnition_switch == SWITCH_OFF)
		{
			return -1;
		}
		ret_speech_recognition=speech_recognition_while(voice.voice_recongnition_text, sizeof(voice.voice_recongnition_text));
		check_state();
		if(ret_speech_recognition!=-2)
		{

			if(strlen(voice.voice_recongnition_text) != 0)
			{
			
				char *psubstr=strstr(voice.voice_recongnition_text,"小雨");
				if(psubstr==NULL)
				{
					psubstr=strstr(voice.voice_recongnition_text,"小米");
					if(psubstr==NULL)
					{
						psubstr=strstr(voice.voice_recongnition_text,"小余");
					}
				}
				
				if(psubstr!=NULL)//含有唤醒词
				{
					printf("====含有唤醒词========psubstr p=%p psubstr=%s strlen(psubstr)=%d  strlen(voice.voice_recongnition_text)=%d\n",psubstr,psubstr,strlen(psubstr),strlen(voice.voice_recongnition_text));
					if(voice.voice_type==1)
					{
						  printf("====含有唤醒词=======\n");
						  pid_t status = system("killall /home/robot/catkin_ws/devel/lib/xfei_asr/tts_speech_lan");
						  printf("system(killall)====exit status value = [0x%x]\n", status);
						  n.setParam("my_param_state", "speech_stop");
					}
					//if(strcmp(voice.voice_recongnition_text+6,"。")!=0)
					if(strcmp(psubstr+6,"。")!=0)
					{
						std_msgs::String msg;
						//msg.data = voice.voice_recongnition_text+6;
						msg.data = psubstr+6;
						pub3.publish(msg);
						n.setParam("my_param_state", "speech_start");
					}		

				}
				else//没有唤醒词
				{
					
					if (voice.sound_box_ongoing_flag ==0&&voice.voice_type==0)//没有语音播报处于空闲
					{
						printf("解析结果send tuling:%s  voice.sound_box_ongoing_flag=%d\n",voice.voice_recongnition_text,voice.sound_box_ongoing_flag);
						std_msgs::String msg;
						msg.data = voice.voice_recongnition_text;
						pub3.publish(msg);
						n.setParam("my_param_state", "speech_start");
					}
				}
			}
		
		}
		else
		{
			//iat_exit:
			if (once_upload_pcm_buffer != NULL)
			{
				free(once_upload_pcm_buffer);
			}
			free(rec_pcm_buffer);	
			snd_pcm_drain(handle);
			snd_pcm_close(handle);
			QISRSessionEnd(session_id, "end");
			MSPLogout(); //退出登录科大讯飞
			//return -1;
			goto continue_error;
		}
		

        ros::spinOnce();
    }

exit:

	MSPLogout(); //退出登录科大讯飞
	show_sys_info("语音识别控制已经关闭...\n");
	return 0;
}

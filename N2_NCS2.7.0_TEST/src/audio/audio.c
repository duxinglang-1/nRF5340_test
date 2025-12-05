/****************************************Copyright (c)************************************************
** File Name:			    audio.c
** Descriptions:			audio process source file
** Created By:				xie biao
** Created Date:			2021-03-04
** Modified Date:      		2021-05-08 
** Version:			    	V1.1
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "audio.h"
#include "uart.h"
#include "logger.h"

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
#define AUDIO_PORT DT_NODELABEL(gpio1)
#else
#error "gpio0 devicetree node is disabled"
#define AUDIO_PORT	""
#endif

#define WTN_DATA	1
#define WTN_BUSY	0

#define AUDIO_VOL_MAX	(0xEF)
#define AUDIO_VOL_MIN	(0xE0)

static bool audio_play_flag = false;
static bool audio_repeat_flag = false;
static bool audio_stop_flag = false;
static bool audio_vol_inc_flag = false;
static bool audio_vol_dec_flag = false;
static bool audio_trige_flag = false;

static uint8_t g_audio_vol = 0xEF;
static uint8_t g_audio_song = 0;

static struct device *gpio_audio;
static struct gpio_callback gpio_cb;

//延时函数
static void Delay_ms(unsigned int dly)
{
	k_sleep(K_MSEC(dly));
}

static void Delay_us(unsigned int dly)
{
	k_sleep(K_USEC(dly));
}

//发送一个字节数据
void Audio_Send_ByteData(uint8_t data)
{
	uint8_t j;
	
	gpio_pin_set(gpio_audio, WTN_DATA, 0);
	Delay_ms(5);
	
	for(j=0;j<8;j++)
	{
		if(data&0x01)
		{
			gpio_pin_set(gpio_audio, WTN_DATA, 1);
			Delay_us(600);
			gpio_pin_set(gpio_audio, WTN_DATA, 0);
			Delay_us(200);
		}
		else
		{
			gpio_pin_set(gpio_audio, WTN_DATA, 1);
			Delay_us(200);
			gpio_pin_set(gpio_audio, WTN_DATA, 0);
			Delay_us(600);
			
		}
		data >>= 1;
	}
	
	gpio_pin_set(gpio_audio, WTN_DATA, 1);
}

//控制音量
void Volume_Control(unsigned char vol)  //E0  ------  EF
{
	Audio_Send_ByteData(vol);
	Delay_us(400);
}

//播放语音
void Voice_Start(uint8_t voice_addr)
{
	Audio_Send_ByteData(voice_addr);
}

//停止播放
void Voice_Stop(void)
{
	Audio_Send_ByteData(0xFE);
}

//循环播放当前语音
void Voice_Loop(void)
{	
	Delay_us(400);
	Audio_Send_ByteData(0xF2);
}

//播放120报警声
void audio_play_alarm(void)
{
	g_audio_song = 3;
	audio_play_flag = true;
}

//播放中文语音提示
void audio_play_chn_voice(void)
{
	g_audio_song = 1;
	audio_play_flag = true;
}

//播放英文语音提示
void audio_play_en_voice(void)
{
	g_audio_song = 2;
	audio_play_flag = true;
}

void audio_vol_inc(void)
{
	audio_vol_inc_flag = true;
}

void audio_vol_dec(void)
{
	audio_vol_dec_flag = true;
}

//停止播放
void audio_stop(void)
{
	audio_stop_flag = true;
}

//SOS停止播放报警
void SOSStopAlarm(void)
{
	audio_stop_flag = true;
}

//SOS播放报警
void SOSPlayAlarm(void)
{
	g_audio_song = 3;
	audio_play_flag = true;
}

//摔倒停止播放报警
void FallStopAlarm(void)
{
	audio_stop_flag = true;
}

//摔倒播放中文报警
void FallPlayAlarmCn(void)
{
	g_audio_song = 1;
	audio_play_flag = true;
}

//摔倒播放英文报警
void FallPlayAlarmEn(void)
{
	g_audio_song = 2;
	audio_play_flag = true;
}

void AudioInterruptHandle(void)
{
	audio_trige_flag = true;
}

void UartAudioEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	
	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, AUDIO_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(AUDIO_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_AUDIO_GET_INFOR)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PLAY)) != NULL)
		{
			uint8_t buffer[8] = {0};
			
			ptr1 += strlen(COM_AUDIO_PLAY);
			memcpy(buffer, ptr1, data_len-(ptr-data));
			g_audio_song = atoi(buffer);
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_STOP)) != NULL)
		{
			audio_stop();
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PAUSE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_RESUME)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_NEXT)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PRE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_VOL_INC)) != NULL)
		{
			audio_vol_inc();
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_VOL_DEC)) != NULL)
		{
			audio_vol_dec();
		}
	}
}

//io口初始化 
void audio_init(void)
{
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_UP;
	
	gpio_audio = DEVICE_DT_GET(AUDIO_PORT);
	if(gpio_audio == NULL)
		return;

	gpio_pin_configure(gpio_audio, WTN_DATA, GPIO_OUTPUT);
	gpio_pin_set(gpio_audio, WTN_DATA, 1);

	//busy interrupt
	gpio_pin_configure(gpio_audio, WTN_BUSY, flag);
    gpio_pin_interrupt_configure(gpio_audio, WTN_BUSY, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, AudioInterruptHandle, BIT(WTN_BUSY));
	gpio_add_callback(gpio_audio, &gpio_cb);
    gpio_pin_interrupt_configure(gpio_audio, WTN_BUSY, GPIO_INT_ENABLE|GPIO_INT_EDGE_RISING);

	Volume_Control(g_audio_vol);
	Delay_ms(100);
}

void AudioMsgProcess(void)
{
	if(audio_play_flag)
	{
		Voice_Start(g_audio_song);
		audio_play_flag = false;
	}

	if(audio_repeat_flag)
	{
		Voice_Loop();
		audio_repeat_flag = false;
	}
	
	if(audio_stop_flag)
	{
		Voice_Stop();
		audio_stop_flag = false;
	}

	if(audio_vol_inc_flag)
	{
		if(g_audio_vol < AUDIO_VOL_MAX)
		{
			g_audio_vol++;
			Volume_Control(g_audio_vol);
		}
		audio_vol_inc_flag = false;
	}

	if(audio_vol_dec_flag)
	{
		if(g_audio_vol > AUDIO_VOL_MIN)
		{
			g_audio_vol--;
			Volume_Control(g_audio_vol);
		}
		audio_vol_dec_flag = false;
	}

	if(audio_trige_flag)
	{
		audio_trige_flag = false;
	}
}

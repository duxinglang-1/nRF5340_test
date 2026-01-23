/****************************************Copyright (c)************************************************
** File Name:			    esp8266.c
** Descriptions:			wifi process source file
** Created By:				xie biao
** Created Date:			2021-03-29
** Modified Date:      		2021-03-29 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include "esp8266.h"
#include "uart.h"
#include "logger.h"
#include "transfer_cache.h"

//#define WIFI_DEBUG

#define WIFI_EN_PIN		6

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
#define WIFI_RST_PORT DT_NODELABEL(gpio0)
#else
#error "gpio0 devicetree node is disabled"
#define WIFI_RST_PORT	""
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
#define WIFI_EN_PORT DT_NODELABEL(gpio1)
#else
#error "gpio0 devicetree node is disabled"
#define WIFI_EN_PORT	""
#endif


#define WIFI_RETRY_COUNT_MAX	5
#define WIFI_AUTO_OFF_TIME_SEC	(1)

uint8_t g_wifi_mac_addr[20] = {0};
uint8_t g_wifi_ver[20] = {0};

static uint8_t retry = 0;

static struct device *gpio_wifi_en = NULL;
static struct device *gpio_wifi_rst = NULL;

bool wifi_is_on = false;

static bool wifi_on_flag = false;
static bool wifi_off_flag = false;
static bool wifi_scan_flag = false;
static bool wifi_rescanning_flag = false;
static bool wifi_off_retry_flag = false;
static bool wifi_off_ok_flag = false;
static bool wifi_get_infor_flag = false;

static void WifiGetInforCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_get_infor_timer, WifiGetInforCallBack, NULL);
static void WifiTurnOffCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_turn_off_timer, WifiTurnOffCallBack, NULL);
static void WifiDelayScanCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_delay_scan_timer, WifiDelayScanCallBack, NULL);


static void WifiGetInforCallBack(struct k_timer *timer_id)
{
	wifi_get_infor_flag = true;
}

static void WifiTurnOffCallBack(struct k_timer *timer_id)
{
	wifi_off_flag = true;
}

static void WifiDelayScanCallBack(struct k_timer *timer_id)
{
	wifi_scan_flag = true;
}

/*============================================================================
* Function Name  : Send_Cmd_To_Esp8285
* Description    : 向ESP8265送命令
* Input          : cmd:发送的命令字符串;waittime:等待时间(单位:ms)
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void Send_Cmd_To_Esp8285(uint8_t *cmd, uint32_t WaitTime)
{
	WifiSendData(cmd, strlen(cmd));

	if(WaitTime > 0)
		k_sleep(K_MSEC(WaitTime));
}

/*============================================================================
* Function Name  : wifi_enable
* Description    : Esp8285_EN使能，低电平有效
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_enable(void)
{
	gpio_pin_set(gpio_wifi_en, WIFI_EN_PIN, 0);
}

/*============================================================================
* Function Name  : wifi_disable
* Description    : Esp8285_EN使能禁止，高电平有效
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_disable(void)
{
	gpio_pin_set(gpio_wifi_en, WIFI_EN_PIN, 1);
}

/*============================================================================
* Function Name  : wifi_start_scanning
* Description    : ESP8285模块启动WiFi信号扫描
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/ 	
void wifi_start_scanning(void)
{
	//设置工作模式 1:station模式 2:AP模式 3:兼容AP+station模式
	Send_Cmd_To_Esp8285(WIFI_SET_MODE, 100);
	//设置AT+CWLAP信号的排序方式：按RSSI排序，只显示信号强度和MAC模式
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_OPT, 50);
	//启动扫描
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_START, 0);
}

/*============================================================================
* Function Name  : wifi_turn_on_and_scanning
* Description    : ESPP8285 init
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_turn_on_and_scanning(void)
{
	wifi_is_on = true;

#ifdef WIFI_DEBUG	
	LOGD("begin");
#endif

	wifi_enable();
	k_timer_start(&wifi_delay_scan_timer, K_MSEC(500), K_NO_WAIT);
}

void wifi_turn_off_success(void)
{
#ifdef WIFI_DEBUG	
	LOGD("begin");
#endif

	gpio_pin_set(gpio_wifi_en, WIFI_EN_PIN, 1);
	wifi_off_retry_flag = false;

	wifi_is_on = false;
	UartWifiOff();
}

void wifi_turn_off(void)
{
#ifdef WIFI_DEBUG
	LOGD("begin");
#endif

	wifi_is_on = false;
	wifi_disable();
	UartWifiOff();
}

void wifi_rescanning(void)
{
	if(!wifi_is_on)
		return;

	//设置AT+CWLAP信号的排序方式：按RSSI排序，只显示信号强度和MAC模式
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_OPT, 50);
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_START, 0);
}

/*============================================================================
* Function Name  : wifi_receive_data_handle
* Description    : NRF9160 接收 ESP8285发来的AP扫描信息进行处理
* Input          : buf:数据缓存 len:数据长度
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_receive_data_handle(uint8_t *buf, uint32_t len)
{
	uint8_t count = 0;
	uint8_t tmpbuf[256] = {0};
	uint8_t *ptr = NULL;
	uint8_t *ptr1 = NULL;
	uint8_t *ptr2 = NULL;
	bool flag = false;

#ifdef WIFI_DEBUG
	LOGD("len:%d, rece:%s", len, buf);
#endif

	if((ptr = strstr(buf, WIFI_SLEEP_REPLY)) != NULL)
	{
		wifi_off_ok_flag = true;
		MapcsSendData(UART_DATA_WIFI, COM_WIFI_CLOSE, strlen(COM_WIFI_CLOSE));
		return;
	}

	if((ptr = strstr(buf, WIFI_GET_MAC_REPLY)) != NULL)
	{
		//AT+CIFSR
		//+CIFSR:STAIP,"192.168.3.221"
		//+CIFSR:STAMAC,"70:03:9f:d3:54:58"
		//\r\n
		//OK
		//\r\n
		ptr1 = strstr(ptr, WIFI_SCAN_DATA_MAC_BEGIN);
		if(ptr1)
		{
			ptr1++;
			ptr2 = strstr(ptr1, WIFI_SCAN_DATA_MAC_BEGIN);
			if(ptr2)
			{
				memcpy(g_wifi_mac_addr, ptr1, ptr2-ptr1);
				sprintf(tmpbuf, "%s%s", COM_WIFI_GET_MAC, g_wifi_mac_addr);
				MapcsSendData(UART_DATA_WIFI, tmpbuf, strlen(tmpbuf));
			}
		}
		return;
	}

	if((ptr = strstr(buf, WIFI_GET_VER)) != NULL)
	{
		//AT+GMR
		//AT version:1.6.2.0(Apr 13 2018 11:10:59)
		//SDK version:2.2.1(6ab97e9)
		//compile time:Jun  7 2018 19:34:26
		//Bin version(Wroom 02):1.6.2
		//OK
		//\r\n
		ptr1 = strstr(ptr, WIFI_GET_DATA_VER_BIN);
		if(ptr1)
		{
			ptr1++;
			ptr1 = strstr(ptr1, WIFI_SCAN_DATA_SEP_COLON);
			if(ptr1)
			{
				ptr1++;
				ptr2 = strstr(ptr1, WIFI_GET_DATA_END);
				if(ptr2)
				{
					memcpy(g_wifi_ver, ptr1, ptr2-ptr1);
					sprintf(tmpbuf, "%s%s", COM_WIFI_GET_VER, g_wifi_ver);
					MapcsSendData(UART_DATA_WIFI, tmpbuf, strlen(tmpbuf));
				}
			}
		}

		wifi_off_flag = true;
		return;
	}

	if((ptr = strstr(buf,WIFI_SCAN_DATA_HEAD)) != NULL)
	{
		//+CWLAP:(-61,"f4:84:8d:8e:9f:eb")
		//+CWLAP:(-67,"da:f1:5b:ff:f2:bc")
		//+CWLAP:(-67,"e2:c1:13:2d:9e:47")
		//+CWLAP:(-73,"7c:94:2a:39:9f:50")
		//+CWLAP:(-76,"52:c2:e8:c6:fa:1e")
		//+CWLAP:(-80,"80:ea:07:73:96:1a")
		//\r\n
		//OK
		//\r\n
		uint8_t *pdata;
		uint32_t com_len = strlen(COM_WIFI_GET_SCAN_AP);

		pdata = k_malloc(com_len+len);
		if(pdata != NULL)
		{
			memset(pdata, 0x00, com_len+len);
			memcpy(pdata, COM_WIFI_GET_SCAN_AP, com_len);
			memcpy(pdata+com_len, buf, len);
			MapcsSendData(UART_DATA_WIFI, pdata, com_len+len);

			k_free(pdata);
		}
	}	
}

void UartWifiEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	uint8_t tmpbuf[256] = {0};

#ifdef WIFI_DEBUG
	LOGD("len:%d, data:%s", data_len, data);
#endif

	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, WIFI_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(WIFI_DATA_HEAD);
		
		if((ptr1 = strstr(ptr, COM_WIFI_GET_VER)) != NULL)
		{
			sprintf(tmpbuf, "%s%s", COM_WIFI_GET_VER, g_wifi_ver);
			MapcsSendData(UART_DATA_WIFI, tmpbuf, strlen(tmpbuf));
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_GET_MAC)) != NULL)
		{
			sprintf(tmpbuf, "%s%s", COM_WIFI_GET_MAC, g_wifi_mac_addr);
			MapcsSendData(UART_DATA_WIFI, tmpbuf, strlen(tmpbuf));
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_GET_SCAN_AP)) != NULL)
		{
			wifi_on_flag = true;
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_GET_RESCAN_AP)) != NULL)
		{
			wifi_rescanning_flag = true;
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_SEARCH_AP)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_CONNECT_AP)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_DISCONNECT_AP)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_CONNECT_SERVER)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_DISCONNECT_SERVER)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_SEND_DATA)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_RECE_DATA)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_WIFI_CLOSE)) != NULL)
		{
			wifi_off_flag = true;
		}
	}
}

void WifiMsgProcess(void)
{
	static uint8_t wifi_sleep_retry = 0;
	
	if(wifi_get_infor_flag)
	{
		wifi_get_infor_flag = false;
		wifi_get_infor();
	}

	if(wifi_scan_flag)
	{
		wifi_scan_flag = false;
		wifi_start_scanning();
	}
	
	if(wifi_on_flag)
	{
		wifi_on_flag = false;

		if(k_timer_remaining_get(&wifi_turn_off_timer) > 0)
			k_timer_stop(&wifi_turn_off_timer);
		
		wifi_turn_on_and_scanning();  
	}
	
	if(wifi_off_flag)
	{
		wifi_off_flag = false;
		wifi_scan_flag = false;
		
		wifi_turn_off();

		if(k_timer_remaining_get(&wifi_delay_scan_timer) > 0)
			k_timer_stop(&wifi_delay_scan_timer);
		if(k_timer_remaining_get(&wifi_turn_off_timer) > 0)
			k_timer_stop(&wifi_turn_off_timer);
	}

	if(wifi_rescanning_flag)
	{
		wifi_rescanning_flag = false;
		wifi_rescanning();
	}

	if(wifi_off_ok_flag)
	{
		wifi_off_ok_flag = false;
		wifi_sleep_retry = 0;
		wifi_turn_off_success();
	}
	
	if(wifi_off_retry_flag)
	{
		wifi_off_retry_flag = false;
		wifi_sleep_retry++;
		if(wifi_sleep_retry > 3)
			wifi_off_ok_flag = true;
		else
			wifi_off_flag = true;
	}
}

void wifi_get_infor(void)
{
	//设置工作模式 1:station模式 2:AP模式 3:兼容AP+station模式
	Send_Cmd_To_Esp8285(WIFI_SET_MODE, 10);
	//获取Mac地址
	Send_Cmd_To_Esp8285(WIFI_GET_MAC_CMD, 10);
	//获取版本信息
	Send_Cmd_To_Esp8285(WIFI_GET_VER, 0);

	k_timer_start(&wifi_turn_off_timer, K_SECONDS(5), K_NO_WAIT);
}

void wifi_init(void)
{
#ifdef WIFI_DEBUG
	LOGD("begin");
#endif

	uart_wifi_init();

	gpio_wifi_en = DEVICE_DT_GET(WIFI_EN_PORT);
	gpio_wifi_rst = DEVICE_DT_GET(WIFI_RST_PORT);

	gpio_pin_configure(gpio_wifi_en, WIFI_EN_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_wifi_en, WIFI_EN_PIN, 0);

	k_timer_start(&wifi_get_infor_timer, K_SECONDS(3), K_NO_WAIT);
}

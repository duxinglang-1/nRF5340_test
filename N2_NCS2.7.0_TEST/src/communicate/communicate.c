/****************************************Copyright (c)************************************************
** File Name:			    communicate.c
** Descriptions:			communicate source file
** Created By:				xie biao
** Created Date:			2021-04-28
** Modified Date:      		2021-04-28 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "settings.h"
#include "uart.h"
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif
#include "datetime.h"
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#ifdef CONFIG_TEMP_SUPPORT
#include "temp.h"
#endif
#ifdef CONFIG_IMU_SUPPORT
#include "Lsm6dso.h"
#endif
#include "communicate.h"
#include "logger.h"

extern uint16_t g_last_steps;

#ifdef CONFIG_WIFI_SUPPORT
/*****************************************************************************
 * FUNCTION
 *  location_get_wifi_data_reply
 * DESCRIPTION
 *  定位协议包获取WiFi数据之后的上传数据包处理
 * PARAMETERS
 *  wifi_data       [IN]       wifi数据结构体
 * RETURNS
 *  Nothing
 *****************************************************************************/
void location_get_wifi_data_reply(wifi_infor wifi_data)
{
	uint8_t reply[256] = {0};
	uint32_t i,count=3;

	if(wifi_data.count > 0)
		count = wifi_data.count;
		
	strcat(reply, "3,");
	for(i=0;i<count;i++)
	{
		strcat(reply, wifi_data.node[i].mac);
		strcat(reply, "&");
		strcat(reply, wifi_data.node[i].rssi);
		strcat(reply, "&");
		if(i < (count-1))
			strcat(reply, "|");
	}

	//NBSendLocationData(reply, strlen(reply));
}
#endif

void TimeCheckSendWristOffData(void)
{
	uint8_t reply[8] = {0};

	if(CheckSCC())
		strcpy(reply, "1");
	else
		strcpy(reply, "0");
	
	//NBSendTimelyWristOffData(reply, strlen(reply));
}

#if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendSportData
 * DESCRIPTION
 *  定时检测并上传运动数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void TimeCheckSendSportData(void)
{
	uint8_t i,tmpbuf[20] = {0};
	uint16_t step_data[24] = {0};
	sleep_data sleep[24] = {0};
	uint8_t reply[512] = {0};

	memset(&reply, 0x00, sizeof(reply));
	
	//wrist
	if(ppg_skin_contacted_flag)
		strcpy(reply, "1,");
	else
		strcpy(reply, "0,");
	
#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
	GetCurDayStepRecData(step_data);
#endif
	for(i=0;i<24;i++)
	{
		uint16_t calorie,distance;
		
		memset(tmpbuf,0,sizeof(tmpbuf));
		
		if(step_data[i] == 0xffff)
			step_data[i] = 0;

		distance = 0.7*step_data[i];
		calorie = (0.8214*60*distance)/1000;
		sprintf(tmpbuf, "%d&%d&%d", step_data[i], distance, calorie);

		if(i<23)
			strcat(tmpbuf,"|");
		else
			strcat(tmpbuf,",");
		
		strcat(reply, tmpbuf);
	}

#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_SLEEP_SUPPORT)
	GetCurDaySleepRecData((uint8_t*)&sleep);
#endif
	for(i=0;i<24;i++)
	{
		uint16_t total_sleep;
		
		memset(tmpbuf,0,sizeof(tmpbuf));
		
		if(sleep[i].deep == 0xffff)
			sleep[i].deep = 0;
		if(sleep[i].light == 0xffff)
			sleep[i].light = 0;

		total_sleep = sleep[i].deep+sleep[i].light;
		sprintf(tmpbuf, "%d&%d&%d", total_sleep, sleep[i].light, sleep[i].deep);

		if(i<23)
			strcat(tmpbuf,"|");
		strcat(reply, tmpbuf);
	}

	//NBSendTimelySportData(reply, strlen(reply));
}
#endif

#ifdef CONFIG_PPG_SUPPORT
/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendHrData
 * DESCRIPTION
 *  定时检测并上传健康数据心率包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
uint8_t hr_data[sizeof(ppg_hr_rec2_data)] = {0};
void TimeCheckSendHrData(void)
{
	uint16_t i,len;
	uint8_t tmpbuf[32] = {0};
	uint8_t reply[2048] = {0};
	hr_rec2_nod *p_hr;

	memset(&hr_data, 0x00, sizeof(hr_data));
	GetCurDayHrRecData(&hr_data);
	p_hr = (hr_rec2_nod*)hr_data;

	for(i=0;i<PPG_REC2_MAX_DAILY;i++)
	{
		memset(tmpbuf,0,sizeof(tmpbuf));

		if((p_hr->year == 0xffff || p_hr->year == 0x0000)
			||(p_hr->month == 0xff || p_hr->month == 0x00)
			||(p_hr->day == 0xff || p_hr->day == 0x00)
			||(p_hr->hour == 0xff || p_hr->min == 0xff)
			)
		{
			break;
		}
		
		sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_hr->year, p_hr->month, p_hr->day, p_hr->hour, p_hr->min);
		strcat(reply, tmpbuf);
		sprintf(tmpbuf, "%d|", p_hr->hr);
		strcat(reply, tmpbuf);

		p_hr++;
	}

	len = strlen(reply);
	if(len > 0)
		reply[len-1] = ',';
	else
		reply[len] = ',';
	//NBSendTimelyHrData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendSpo2Data
 * DESCRIPTION
 *  定时检测并上传健康数据血氧包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
uint8_t spo2_data[sizeof(ppg_spo2_rec2_data)] = {0};
void TimeCheckSendSpo2Data(void)
{
	uint16_t i,len;
	uint8_t tmpbuf[32] = {0};
	uint8_t reply[2048] = {0};
	spo2_rec2_nod *p_spo2;

	memset(&spo2_data, 0x00, sizeof(spo2_data));
	GetCurDaySpo2RecData(&spo2_data);
	p_spo2 = (spo2_rec2_nod*)spo2_data;

	for(i=0;i<PPG_REC2_MAX_DAILY;i++)
	{
		memset(tmpbuf,0,sizeof(tmpbuf));

		if((p_spo2->year == 0xffff || p_spo2->year == 0x0000)
			||(p_spo2->month == 0xff || p_spo2->month == 0x00)
			||(p_spo2->day == 0xff || p_spo2->day == 0x00)
			||(p_spo2->hour == 0xff || p_spo2->min == 0xff)
			)
		{
			break;
		}
		
		sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_spo2->year, p_spo2->month, p_spo2->day, p_spo2->hour, p_spo2->min);
		strcat(reply, tmpbuf);
		sprintf(tmpbuf, "%d|", p_spo2->spo2);
		strcat(reply, tmpbuf);

		p_spo2++;
	}

	len = strlen(reply);
	if(len > 0)
		reply[len-1] = ',';
	else
		reply[len] = ',';
	//NBSendTimelySpo2Data(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendBptData
 * DESCRIPTION
 *  定时检测并上传健康数据血压包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
uint8_t bp_data[sizeof(ppg_bpt_rec2_data)] = {0};
void TimeCheckSendBptData(void)
{
	uint16_t i,len;
	uint8_t tmpbuf[32] = {0};
	uint8_t reply[2048] = {0};
	bpt_rec2_nod *p_bpt;

	memset(&bp_data, 0x00, sizeof(bp_data));
	GetCurDayBptRecData(&bp_data);
	p_bpt = (bpt_rec2_nod*)bp_data;

	for(i=0;i<PPG_REC2_MAX_DAILY;i++)
	{
		memset(tmpbuf,0,sizeof(tmpbuf));

		if((p_bpt->year == 0xffff || p_bpt->year == 0x0000)
			||(p_bpt->month == 0xff || p_bpt->month == 0x00)
			||(p_bpt->day == 0xff || p_bpt->day == 0x00)
			||(p_bpt->hour == 0xff || p_bpt->min == 0xff)
			)
		{
			break;
		}
		
		sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_bpt->year, p_bpt->month, p_bpt->day, p_bpt->hour, p_bpt->min);
		strcat(reply, tmpbuf);
		sprintf(tmpbuf, "%d&%d|", p_bpt->bpt.systolic, p_bpt->bpt.diastolic);
		strcat(reply, tmpbuf);

		p_bpt++;
	}

	len = strlen(reply);
	if(len > 0)
		reply[len-1] = ',';
	else
		reply[len] = ',';
	//NBSendTimelyBptData(reply, strlen(reply));
}
#endif

#ifdef CONFIG_TEMP_SUPPORT
/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendTempData
 * DESCRIPTION
 *  定时检测并上传健康数据体温包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
uint8_t temp_data[sizeof(temp_rec2_data)] = {0};
void TimeCheckSendTempData(void)
{
	uint16_t i,len;
	uint8_t tmpbuf[32] = {0};
	uint8_t reply[2048] = {0};
	temp_rec2_nod *p_temp;

	memset(&temp_data, 0x00, sizeof(temp_data));
	GetCurDayTempRecData(&temp_data);
	p_temp = (temp_rec2_nod*)temp_data;
	
	for(i=0;i<TEMP_REC2_MAX_DAILY;i++)
	{
		memset(tmpbuf,0,sizeof(tmpbuf));

		if((p_temp->year == 0xffff || p_temp->year == 0x0000)
			||(p_temp->month == 0xff || p_temp->month == 0x00)
			||(p_temp->day == 0xff || p_temp->day == 0x00)
			||(p_temp->hour == 0xff || p_temp->min == 0xff)
			)
		{
			break;
		}
		
		sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_temp->year, p_temp->month, p_temp->day, p_temp->hour, p_temp->min);
		strcat(reply, tmpbuf);
		sprintf(tmpbuf, "%0.1f|", (float)p_temp->deca_temp/10.0);
		strcat(reply, tmpbuf);

		p_temp++;
	}

	len = strlen(reply);
	if(len > 0)
		reply[len-1] = ',';
	else
		reply[len] = ',';
	//NBSendTimelyTempData(reply, strlen(reply));
}
#endif

/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendHealthData
 * DESCRIPTION
 *  定时检测并上传健康数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/ 
void TimeCheckSendHealthData(void)
{
	if(1
		#ifdef CONFIG_PPG_SUPPORT	
		 && CheckSCC()
		#endif	
		)
	{
	#ifdef CONFIG_PPG_SUPPORT
		TimeCheckSendHrData();
		TimeCheckSendSpo2Data();
		TimeCheckSendBptData();
	#endif	
	#ifdef CONFIG_TEMP_SUPPORT	
		TimeCheckSendTempData();
	#endif
	}
	else
	{
		TimeCheckSendWristOffData();
	}
}

#if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
/*****************************************************************************
 * FUNCTION
 *  SendMissingSportData
 * DESCRIPTION
 *  补发漏传的运动数据包(不补发当天的数据，防止固定的23点的时间戳造成当天数据混乱)
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendMissingSportData(void)
{
	uint8_t i,j,tmpbuf[20] = {0};
	uint8_t stepbuf[STEP_REC2_DATA_SIZE] = {0};
	uint8_t sleepbuf[SLEEP_REC2_DATA_SIZE] = {0};	

#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
	SpiFlash_Read(stepbuf, STEP_REC2_DATA_ADDR, STEP_REC2_DATA_SIZE);
#endif
#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_SLEEP_SUPPORT)
	SpiFlash_Read(sleepbuf, SLEEP_REC2_DATA_ADDR, SLEEP_REC2_DATA_SIZE);
#endif

	for(i=0;i<7;i++)
	{
		bool flag = false;
		uint8_t reply[2048] = {0};
		uint16_t step_data[24] = {0};
		uint8_t step_time[15] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x00};
		sleep_data sleep[24] = {0};
		uint8_t sleep_time[15] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x00};
		step_rec2_data step_rec2 = {0};
		sleep_rec2_data	sleep_rec2 = {0};

	#if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
	  #if defined(CONFIG_STEP_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
	  	//step
		memcpy(&step_rec2, &stepbuf[i*sizeof(step_rec2_data)], sizeof(step_rec2_data));
		if((step_rec2.year != 0xffff && step_rec2.year != 0x0000)
			&&(step_rec2.month != 0xff && step_rec2.month != 0x00)
			&&(step_rec2.day != 0xff && step_rec2.day != 0x00)
			)
		{
			flag = true;
			memcpy(step_data, step_rec2.steps, sizeof(step_rec2.steps));
			sprintf(step_time, "%04d%02d%02d230000", step_rec2.year,step_rec2.month,step_rec2.day);
		}
	  #endif
	  #if defined(CONFIG_SLEEP_SUPPORT)&&defined(CONFIG_SLEEP_SUPPORT)
		//sleep
		memcpy(&sleep_rec2, &sleepbuf[i*sizeof(sleep_rec2_data)], sizeof(sleep_rec2_data));
	  	if((sleep_rec2.year != 0xffff && sleep_rec2.year != 0x0000)
			&&(sleep_rec2.month != 0xff && sleep_rec2.month != 0x00)
			&&(sleep_rec2.day != 0xff && sleep_rec2.day != 0x00)
			)
		{
			flag = true;
			memcpy(sleep, sleep_rec2.sleep, sizeof(sleep_rec2.sleep));
			sprintf(sleep_time, "%04d%02d%02d230000", sleep_rec2.year,sleep_rec2.month,sleep_rec2.day);
		}
	  #endif
	#endif
	
		if(!flag)
			continue;
		
		//step
		for(j=0;j<24;j++)
		{
			uint16_t calorie,distance;
			
			memset(tmpbuf,0,sizeof(tmpbuf));
			
			if(step_data[j] == 0xffff)
				step_data[j] = 0;

			distance = 0.7*step_data[j];
			calorie = (0.8214*60*distance)/1000;
			sprintf(tmpbuf, "%d&%d&%d", step_data[j], distance, calorie);

			if(j<23)
				strcat(tmpbuf,"|");
			else
				strcat(tmpbuf,",");
			strcat(reply, tmpbuf);
		}
		strcat(reply, step_time);
		strcat(reply, ",");
		
		//sleep
		for(j=0;j<24;j++)
		{
			uint16_t total_sleep;
			
			memset(tmpbuf,0,sizeof(tmpbuf));
			
			if(sleep[j].deep == 0xffff)
				sleep[j].deep = 0;
			if(sleep[j].light == 0xffff)
				sleep[j].light = 0;

			total_sleep = sleep[j].deep+sleep[j].light;
			sprintf(tmpbuf, "%d&%d&%d", total_sleep, sleep[j].light, sleep[j].deep);

			if(j<23)
				strcat(tmpbuf,"|");
			else
				strcat(tmpbuf,",");
			strcat(reply, tmpbuf);
		}
		strcat(reply, sleep_time);

		//NBSendMissSportData(reply, strlen(reply));
	}
}
#endif

/*****************************************************************************
 * FUNCTION
 *  TimeCheckSendLocationData
 * DESCRIPTION
 *  定时检测并上传定位数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void TimeCheckSendLocationData(void)
{
	static uint32_t loc_hour_count = 0;
	bool flag = false;

	loc_hour_count++;
	if(date_time.hour >= 21 || date_time.hour < 9)
	{
		if(loc_hour_count == 360)
		{
			flag = true;
		}
	}
	else if(loc_hour_count == global_settings.dot_interval.time)
	{
		flag = true;
	}

	if(flag)
	{
		loc_hour_count = 0;
	#ifdef CONFIG_WIFI_SUPPORT
		location_wait_wifi = true;
		APP_Ask_wifi_data();
	#else
		location_wait_gps = true;
		APP_Ask_GPS_Data();
	#endif
	}
}

/*****************************************************************************
 * FUNCTION
 *  StepCheckSendLocationData
 * DESCRIPTION
 *  计步检测并上传定位数据包
 * PARAMETERS
 *	steps			[IN]		当前累计的记步数
 * RETURNS
 *  Nothing
 *****************************************************************************/
void StepCheckSendLocationData(uint16_t steps)
{
	static uint16_t step_count = 0;

	if(step_count == 0)
		step_count = g_last_steps;
	
	if((steps - step_count) >= global_settings.dot_interval.steps)
	{
		step_count = steps;

	#ifdef CONFIG_WIFI_SUPPORT
		location_wait_wifi = true;
		APP_Ask_wifi_data();
	#else
		location_wait_gps = true;
		APP_Ask_GPS_Data();
	#endif		
	}
}

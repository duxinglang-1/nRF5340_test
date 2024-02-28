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
#include "external_flash.h"
#include "uart_ble.h"
#include "datetime.h"
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#ifdef CONFIG_IMU_SUPPORT
#include "Lsm6dso.h"
#endif
#include "communicate.h"
#include "logger.h"

extern uint16_t g_last_steps;

void TimeCheckSendWristOffData(void)
{
	uint8_t reply[8] = {0};

	if(CheckSCC())
		strcpy(reply, "1");
	else
		strcpy(reply, "0");
	
	//NBSendTimelyWristOffData(reply, strlen(reply));
}

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
	if(CheckSCC())
	{
		TimeCheckSendHrData();
		TimeCheckSendSpo2Data();
		TimeCheckSendBptData();
	}
	else
	{
		TimeCheckSendWristOffData();
	}
}

#if defined(CONFIG_PPG_SUPPORT)
static uint8_t databuf[PPG_REC2_MAX_DAILY*sizeof(bpt_rec2_nod)] = {0};
/*****************************************************************************
 * FUNCTION
 *  SendMissingHrData
 * DESCRIPTION
 *  补发漏传的健康数据心率包(不补发当天的数据，防止固定的23点的时间戳造成当天数据混乱)
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendMissingHrData(void)
{
	uint16_t i,j,len;
	uint8_t tmpbuf[32] = {0};
	hr_rec2_nod *p_hr;
	sys_date_timer_t temp_date = {0};

	memcpy(&temp_date, &date_time, sizeof(sys_date_timer_t));
	DateDecrease(&temp_date, 6);
	
	for(i=0;i<7;i++)
	{
		bool flag = false;
		uint8_t hr_time[15] = {0x30};
		uint8_t reply[2048] = {0};

		memset(&databuf, 0x00, sizeof(databuf));
		GetGivenDayHrRecData(temp_date, &databuf);
		p_hr = (hr_rec2_nod*)databuf;
		for(j=0;j<PPG_REC2_MAX_DAILY;j++)
		{
		  	if((p_hr->year == 0xffff || p_hr->year == 0x0000)
				||(p_hr->month == 0xff || p_hr->month == 0x00)
				||(p_hr->day == 0xff || p_hr->day == 0x00)
				||(p_hr->hour == 0xff || p_hr->min == 0xff)
				)
			{
				break;
			}

			if(!flag)
			{
				flag = true;
				sprintf(hr_time, "%04d%02d%02d230000", p_hr->year, p_hr->month, p_hr->day);
			}

			memset(tmpbuf,0,sizeof(tmpbuf));
			sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_hr->year, p_hr->month, p_hr->day, p_hr->hour, p_hr->min);
			strcat(reply, tmpbuf);
			sprintf(tmpbuf, "%d|", p_hr->hr);
			strcat(reply, tmpbuf);
			
			p_hr++;
		}

		len = strlen(reply);
		if(len > 0)
		{
			reply[len-1] = ',';
			strcat(reply, hr_time);
			//NBSendMissHrData(reply, strlen(reply));
		}

		DateIncrease(&temp_date, 1);
	}
}

/*****************************************************************************
 * FUNCTION
 *  SendMissingSpo2Data
 * DESCRIPTION
 *  补发漏传的健康数据血氧包(不补发当天的数据，防止固定的23点的时间戳造成当天数据混乱)
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendMissingSpo2Data(void)
{
	uint16_t i,j,len;
	uint8_t tmpbuf[32] = {0};
	spo2_rec2_nod *p_spo2;
	sys_date_timer_t temp_date = {0};

	memcpy(&temp_date, &date_time, sizeof(sys_date_timer_t));
	DateDecrease(&temp_date, 6);
	
	for(i=0;i<7;i++)
	{
		bool flag = false;
		uint8_t spo2_time[15] = {0x30};
		uint8_t reply[2048] = {0};

		memset(&databuf, 0x00, sizeof(databuf));
		GetGivenDaySpo2RecData(temp_date, &databuf);
		p_spo2 = (spo2_rec2_nod*)databuf;
		for(j=0;j<PPG_REC2_MAX_DAILY;j++)
		{
			if((p_spo2->year == 0xffff || p_spo2->year == 0x0000)
				||(p_spo2->month == 0xff || p_spo2->month == 0x00)
				||(p_spo2->day == 0xff || p_spo2->day == 0x00)
				||(p_spo2->hour == 0xff || p_spo2->min == 0xff)
				)
			{
				break;
			}

			if(!flag)
			{
				flag = true;
				sprintf(spo2_time, "%04d%02d%02d230000", p_spo2->year, p_spo2->month, p_spo2->day);
			}

			memset(tmpbuf,0,sizeof(tmpbuf));
			sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_spo2->year, p_spo2->month, p_spo2->day, p_spo2->hour, p_spo2->min);
			strcat(reply, tmpbuf);
			sprintf(tmpbuf, "%d|", p_spo2->spo2);
			strcat(reply, tmpbuf);
			
			p_spo2++;
		}

		len = strlen(reply);
		if(len > 0)
		{
			reply[len-1] = ',';
			strcat(reply, spo2_time);
			//NBSendMissSpo2Data(reply, strlen(reply));
		}

		DateIncrease(&temp_date, 1);
	}
}

/*****************************************************************************
 * FUNCTION
 *  SendMissingBptData
 * DESCRIPTION
 *  补发漏传的健康数据血压包(不补发当天的数据，防止固定的23点的时间戳造成当天数据混乱)
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendMissingBptData(void)
{
	uint16_t i,j,len;
	uint8_t tmpbuf[32] = {0};
	bpt_rec2_nod *p_bpt;
	sys_date_timer_t temp_date = {0};

	memcpy(&temp_date, &date_time, sizeof(sys_date_timer_t));
	DateDecrease(&temp_date, 6);

	for(i=0;i<7;i++)
	{
		bool flag = false;
		uint8_t bpt_time[15] = {0x30};
		uint8_t reply[2048] = {0};

		memset(&databuf, 0x00, sizeof(databuf));
		GetGivenDayBptRecData(temp_date, &databuf);
		p_bpt = (bpt_rec2_nod*)databuf;
		for(j=0;j<PPG_REC2_MAX_DAILY;j++)
		{
			if((p_bpt->year == 0xffff || p_bpt->year == 0x0000)
				||(p_bpt->month == 0xff || p_bpt->month == 0x00)
				||(p_bpt->day == 0xff || p_bpt->day == 0x00)
				||(p_bpt->hour == 0xff || p_bpt->min == 0xff)
				)
			{
				break;
			}

			if(!flag)
			{
				flag = true;
				sprintf(bpt_time, "%04d%02d%02d230000", p_bpt->year, p_bpt->month, p_bpt->day);
			}

			memset(tmpbuf,0,sizeof(tmpbuf));
			sprintf(tmpbuf, "%04d%02d%02d%02d%02d;", p_bpt->year, p_bpt->month, p_bpt->day, p_bpt->hour, p_bpt->min);
			strcat(reply, tmpbuf);
			sprintf(tmpbuf, "%d&%d|", p_bpt->bpt.systolic, p_bpt->bpt.diastolic);
			strcat(reply, tmpbuf);
			
			p_bpt++;
		}

		len = strlen(reply);
		if(len > 0)
		{
			reply[len-1] = ',';
			strcat(reply, bpt_time);
			//NBSendMissBptData(reply, strlen(reply));
		}

		DateIncrease(&temp_date, 1);
	}
}

/*****************************************************************************
 * FUNCTION
 *  SendMissingHealthData
 * DESCRIPTION
 *  补发漏传的健康数据包(不补发当天的数据，防止固定的23点的时间戳造成当天数据混乱)
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendMissingHealthData(void)
{
	SendMissingHrData();
	SendMissingSpo2Data();
	SendMissingBptData();
}

#endif/*CONFIG_PPG_SUPPORT*/
/*****************************************************************************
 * FUNCTION
 *  SyncSendHealthData
 * DESCRIPTION
 *  数据同步上传健康数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SyncSendHealthData(void)
{
	uint16_t steps=0,calorie=0,distance=0;
	uint16_t light_sleep=0,deep_sleep=0;
	uint8_t tmpbuf[20] = {0};
	uint8_t reply[1024] = {0};

#ifdef CONFIG_IMU_SUPPORT
  #ifdef CONFIG_STEP_SUPPORT
	GetSportData(&steps, &calorie, &distance);
  #endif
  #ifdef CONFIG_SLEEP_SUPPORT
	GetSleepTimeData(&deep_sleep, &light_sleep);
  #endif
#endif

	//steps
	sprintf(tmpbuf, "%d,", steps);
	strcpy(reply, tmpbuf);
	
	//wrist
	if(ppg_skin_contacted_flag)
		strcat(reply, "1,");
	else
		strcat(reply, "0,");
	
	//sitdown time
	strcat(reply, "0,");
	
	//activity time
	strcat(reply, "0,");
	
	//light sleep time
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", light_sleep);
	strcat(reply, tmpbuf);
	
	//deep sleep time
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", deep_sleep);
	strcat(reply, tmpbuf);
	
	//move body
	strcat(reply, "0,");
	
	//calorie
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", calorie);
	strcat(reply, tmpbuf);
	
	//distance
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", distance);
	strcat(reply, tmpbuf);

#ifdef CONFIG_PPG_SUPPORT	
	//systolic
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", g_bpt_menu.systolic);
	strcat(reply, tmpbuf);
	
	//diastolic
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", g_bpt_menu.diastolic); 	
	strcat(reply, tmpbuf);
	
	//heart rate
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", g_hr_menu);		
	strcat(reply, tmpbuf);
	
	//SPO2
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%d,", g_spo2_menu); 	
	strcat(reply, tmpbuf);
#else
	strcat(reply, "0,0,0,0,");
#endif/*CONFIG_PPG_SUPPORT*/

#ifdef CONFIG_TEMP_SUPPORT
	//body temp
	memset(tmpbuf,0,sizeof(tmpbuf));
	sprintf(tmpbuf, "%0.1f", g_temp_menu); 	
	strcat(reply, tmpbuf);
#else
	strcat(reply, "0.0");
#endif/*CONFIG_TEMP_SUPPORT*/

	//NBSendSingleHealthData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendPowerOnData
 * DESCRIPTION
 *  发送开机数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendPowerOnData(void)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[256] = {0};

	//imsi
	strcpy(reply, ",");
	
	//iccid
	strcat(reply, ",");

	//nb rsrp
	strcat(reply, ",");
	
	//time zone
	strcat(reply, ",");
	
	//battery
	GetBatterySocString(tmpbuf);
	strcat(reply, tmpbuf);
	strcat(reply, ",");

	//mcu fw version
	strcat(reply, g_fw_version);	
	strcat(reply, ",");

	//modem fw version
	strcat(reply, ",");

	//ppg algo
#ifdef CONFIG_PPG_SUPPORT	
	strcat(reply, g_ppg_ver);
#endif
	strcat(reply, ",");

	//wifi version
	strcat(reply, ",");

	//wifi mac
	strcat(reply, ",");

	//ble version
	strcat(reply, &g_nrf52810_ver[15]);	
	strcat(reply, ",");

	//ble mac
	strcat(reply, g_ble_mac_addr);	

	//NBSendPowerOnInfor(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendPowerOffData
 * DESCRIPTION
 *  发送关机数据包
 * PARAMETERS
 *	pwroff_mode			[IN]		关机模式 1:低电关机 2:按键关机 3:重启关机 
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendPowerOffData(uint8_t pwroff_mode)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[128] = {0};

	//pwr off mode
	sprintf(reply, "%d,", pwroff_mode);
	
	//nb rsrp
	sprintf(tmpbuf, "%d,", 0);
	strcat(reply, tmpbuf);
	
	//battery
	GetBatterySocString(tmpbuf);
	strcat(reply, tmpbuf);
			
	//NBSendPowerOffInfor(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendSettingsData
 * DESCRIPTION
 *  发送终端设置项数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendSettingsData(void)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[128] = {0};

	//temp uint
	sprintf(reply, "%d,", global_settings.temp_unit);
	
	//language
	switch(global_settings.language)
	{
#ifndef FW_FOR_CN
	case LANGUAGE_EN:	//English
		strcat(reply, "en");
		break;
	case LANGUAGE_DE:	//Deutsch
		strcat(reply, "de");
		break;
	case LANGUAGE_FR:	//French
		strcat(reply, "fr");
		break;
	case LANGUAGE_ITA:	//Italian
		strcat(reply, "it");
		break;
	case LANGUAGE_ES:	//Spanish
		strcat(reply, "es");
		break;
	case LANGUAGE_PT:	//Portuguese
		strcat(reply, "pt");
		break;
#else
	case LANGUAGE_CHN:	//Chinese
		strcat(reply, "zh");
		break;
	case LANGUAGE_EN:	//English
		strcat(reply, "en");
		break;
#endif	
	}
	
	//NBSendSettingsData(reply, strlen(reply));
}

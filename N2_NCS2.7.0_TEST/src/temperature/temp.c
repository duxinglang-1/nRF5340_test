/****************************************Copyright (c)************************************************
** File Name:			    temp.c
** Descriptions:			temperature message process source file
** Created By:				xie biao
** Created Date:			2021-12-24
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "uart.h"
#include "temp.h"
#include "inner_flash.h"
#include "logger.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#if defined(TEMP_GXTS04)
#include "gxts04.h"
#elif defined(TEMP_MAX30208)
#include "max30208.h"
#elif defined(TEMP_CT1711)
#include "ct1711.h"
#endif

static bool temp_check_ok = false;
static bool temp_get_data_flag = false;
static bool temp_start_flag = false;
static bool temp_stop_flag = false;
static bool temp_power_flag = false;
static bool menu_start_temp = false;
static bool ft_start_temp = false;

static uint16_t sensor_id;

uint8_t g_temp_trigger = 0;

static void temp_get_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(temp_check_timer, temp_get_timerout, NULL);

static void temp_get_timerout(struct k_timer *timer_id)
{
	temp_get_data_flag = true;
}

void TempStop(void)
{
	temp_stop_flag = true;
}

void StartTemp(TEMP_TRIGGER_SOUCE trigger_type)
{
	g_temp_trigger |= trigger_type;
	temp_start_flag = true;
}

void MenuStartTemp(void)
{
	menu_start_temp = true;
}

void MenuStopTemp(void)
{
	temp_stop_flag = true;
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartTemp(void)
{
	ft_start_temp = true;
}

void FTStopTemp(void)
{
	temp_stop_flag = true;
}
#endif

void UartTempEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	
	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, TEMP_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(TEMP_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_TEMP_GET_INFOR)) != NULL)
		{
			uint8_t buffer[16] = {0};
			uint32_t len;
			
			strcpy(buffer, COM_TEMP_GET_INFOR);
			len = strlen(COM_TEMP_GET_INFOR);
			memcpy(&buffer[len], &sensor_id, sizeof(sensor_id));
			MapcsSendData(COM_TEMP_GET_INFOR, buffer, len+sizeof(sensor_id));
		}
		else if((ptr1 = strstr(ptr, COM_OPEN)) != NULL)
		{
			StartTemp(TEMP_TRIGGER_BY_MENU);
		}
		else if((ptr1 = strstr(ptr, COM_TEMP_GET_DATA)) != NULL)
		{
			StartTemp(TEMP_TRIGGER_BY_MENU);
		}
		else if((ptr1 = strstr(ptr, COM_CLOSE)) != NULL)
		{
			TempStop();
		}
	}
}

void temp_init(void)
{
#ifdef TEMP_GXTS04
	temp_check_ok = gxts04_init(&sensor_id);
#elif defined(TEMP_MAX30208)
	temp_check_ok = max30208_init();
#elif defined(TEMP_CT1711)
	temp_check_ok = ct1711_init();
#endif
}

void TempMsgProcess(void)
{
	if(menu_start_temp)
	{
		StartTemp(TEMP_TRIGGER_BY_MENU);
		menu_start_temp = false;
	}

	if(ft_start_temp)
	{
		StartTemp(TEMP_TRIGGER_BY_FT);
		ft_start_temp = false;
	}
	
	if(temp_start_flag)
	{
		temp_start_flag = false;
		if(temp_power_flag)
			return;
		
	#ifdef TEMP_GXTS04	
		gxts04_start();
	#endif
		temp_power_flag = true;
	
		k_timer_start(&temp_check_timer, K_MSEC(1*1000), K_MSEC(1*1000));
	}

	if(temp_stop_flag)
	{
		temp_stop_flag = false;
		if(!temp_power_flag)
			return;
		
	#ifdef TEMP_GXTS04	
		gxts04_stop();
	#endif
	
		temp_power_flag = false;
		k_timer_stop(&temp_check_timer);

	#ifdef CONFIG_BLE_SUPPORT
		if((g_temp_trigger&TEMP_TRIGGER_BY_APP) != 0)
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_APP);
	#endif	
		if((g_temp_trigger&TEMP_TRIGGER_BY_MENU) != 0)
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_MENU);
		if((g_temp_trigger&TEMP_TRIGGER_BY_HOURLY) != 0)
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_HOURLY);
	#ifdef CONFIG_FACTORY_TEST_SUPPORT	
		if((g_temp_trigger&TEMP_TRIGGER_BY_FT) != 0)
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_FT);
	#endif
	}

	if(temp_get_data_flag)
	{
		uint8_t data[10] = {0};
		uint8_t buffer[64] = {0};
		uint32_t len;
		
		temp_get_data_flag = false;

		if(!temp_check_ok || !temp_power_flag)
			return;

		GetTemperature(&data);
		strcpy(buffer, COM_TEMP_GET_DATA);
		len = strlen(COM_TEMP_GET_DATA);
		memcpy(&buffer[len], data, sizeof(data));
		MapcsSendData(UART_DATA_TEMP, buffer, len+sizeof(data));
	}
}


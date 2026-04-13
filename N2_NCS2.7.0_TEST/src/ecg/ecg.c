/****************************************Copyright (c)************************************************
** File Name:			    ecg.c
** Descriptions:			ecg function main source file
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include "logger.h"
#include "uart.h"
#include "ecg.h"
#if defined(ECG_ADS1292)
#include "ads1292.h"
#elif defined(ECG_MAX86176)
#include "max86176.h"
#endif

void ECG_Start(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_start();
#elif defined(ECG_MAX86176)
	Max86176_start();
#endif
}

void ECG_Stop(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_stop();
#elif defined(ECG_MAX86176)
	Max86176_stop();
#endif
}

void ECGDataProcess(uint8_t *data, uint32_t data_len)
{
}

void UartECGEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	static uint32_t page_num=0,flash_partial=0;

	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, ECG_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(ECG_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_ECG_SET_OPEN)) != NULL)
		{
      		ECG_Start();
		}
		else if((ptr1 = strstr(ptr, COM_ECG_SET_CLOSE)) != NULL)
		{
			ECG_Stop();
		}
	}
}

void ECG_init(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Init();
#elif defined(ECG_MAX86176)
	Max86176_init();
#endif
}

void ECGMsgProcess(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Msg_Process();
#elif defined(ECG_MAX86176)
	Max86176_Msg_Process();
#endif
}

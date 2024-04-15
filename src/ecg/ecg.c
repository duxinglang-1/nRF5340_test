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
#include "ecg.h"
#ifdef ECG_ADS1292
#include "ads1292.h"
#endif

void ECG_init(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Init();
#endif
}

void ECGMsgProcess(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Msg_Process();
#endif
}

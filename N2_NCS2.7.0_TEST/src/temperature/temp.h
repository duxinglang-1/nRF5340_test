/****************************************Copyright (c)************************************************
** File Name:			    temp.h
** Descriptions:			temperature message process head file
** Created By:				xie biao
** Created Date:			2021-12-24
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __TEMP_H__
#define __TEMP_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <math.h>
#include "datetime.h"

//sensor mode
#define TEMP_GXTS04
//#define TEMP_MAX30208
//#define TEMP_CT1711

//sensor interface type
#define TEMP_IF_I2C
#define TEMP_IF_SINGLE_LINE

#define TEMP_CHECK_MENU				60
#define TEMP_CHECK_TIMELY			2
#ifndef CONFIG_PPG_SUPPORT
#define PPG_CHECK_HR_TIMELY			0
#define PPG_CHECK_SPO2_TIMELY		0
#define PPG_CHECK_BPT_TIMELY		0
#endif

#define TEMP_MAX			420
#define TEMP_MIN			340

#define TEMP_REC2_MAX_HOURLY	(4)
#define TEMP_REC2_MAX_DAILY		(TEMP_REC2_MAX_HOURLY*24)
#define TEMP_REC2_MAX_COUNT		(TEMP_REC2_MAX_DAILY*7)

#define COM_TEMP_SET_OPEN		"OPEN:"
#define COM_TEMP_SET_CLOSE		"CLOSE:"
#define COM_TEMP_GET_INFOR		"INFOR:"
#define COM_TEMP_GET_DATA		"TEMP_DATA:"

//#define TEMP_DEBUG

//sensor trigger type
typedef enum
{
	TEMP_TRIGGER_BY_MENU	=	0x01,
	TEMP_TRIGGER_BY_APP		=	0x02,
	TEMP_TRIGGER_BY_HOURLY	=	0x04,
	TEMP_TRIGGER_BY_FT		=	0x08,
}TEMP_TRIGGER_SOUCE;

extern void StartTemp(TEMP_TRIGGER_SOUCE trigger_type);
#endif/*__TEMP_H__*/


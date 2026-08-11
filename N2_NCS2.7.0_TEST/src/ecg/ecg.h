/****************************************Copyright (c)************************************************
** File Name:			    ecg.h
** Descriptions:			ecg function main head file
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __ECG_H__
#define __ECG_H__

#define COM_ECG_SET_OPEN		"OPEN:"
#define COM_ECG_SET_CLOSE		"CLOSE:"
#define COM_ECG_GET_INFOR		"INFOR:"
#define COM_ECG_GET_DATA		"ECG_DATA:"
#define COM_ECG_LEAD_STATUS		"LEAD_STATUS:"
#define COM_ECG_LEAD_OFF		"LEAD_OFF:"
#define COM_ECG_LEAD_ON		    "LEAD_ON:"
#define COM_ECG_LEAD_TIME_OUT	"LEAD_TIME_OUT:"
#define COM_ECG_WEAR_STATUS		"WEAR_STATUS:"
#define COM_ECG_WEAR_LEFT		"WEAR_LEFT:"
#define COM_ECG_WEAR_RIGHT		"WEAR_RIGHT:"
#define COM_ECG_HR_DATA			"ECG_HR:"
#define COM_ECG_HRV_DATA		"ECG_HRV:"
#define COM_ECG_Z_DATA			"ECG_Z:"

// HR 和 HRV 相关配置
#define ECG_SAMPLE_RATE			128		// 采样率 128Hz
#define HRV_RR_BUFFER_SIZE		32		// RR 间期缓冲区大小
#define HR_MIN					30		// 最小心率 (bpm)
#define HR_MAX					220		// 最大心率 (bpm)

// 函数声明
void ECG_HR_HRV_Reset(void);
void ECG_Process_QRS_Detection(short ecg_sample);
uint8_t ECG_Get_Heart_Rate(void);
uint16_t ECG_Get_HRV_SDNN(void);

//#define ECG_ADS1292
#define ECG_MAX86176
#endif/*__ECG_H__*/
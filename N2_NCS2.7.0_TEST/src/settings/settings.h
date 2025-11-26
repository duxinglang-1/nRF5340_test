#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#define ALARM_MAX	8

#define VERSION_STR	"3.4.6_51028"

typedef enum
{
	RESET_STATUS_IDLE,
	RESET_STATUS_RUNNING,
	RESET_STATUS_SUCCESS,
	RESET_STATUS_FAIL,
	RESET_STATUS_MAX
}RESET_STATUS;

typedef enum
{
	SETTINGS_STATUS_INIT,
	SETTINGS_STATUS_OTA,
	SETTINGS_STATUS_NORMAL
}SETTINGS_STATUS;

typedef struct
{
	bool is_on;
	uint8_t hour;
	uint8_t minute;
	uint8_t repeat;	//全是1就是每天提醒，全是0就是只提醒一次，0x1111100就是工作日提醒，其他就是自定义
}alarm_infor_t;

typedef struct
{
	bool is_on;
	uint8_t interval;
}phd_measure_t;		//整点测量

typedef struct
{
	uint32_t steps;
	uint32_t time;
}location_interval_t;

typedef struct
{
	uint8_t systolic;		//收缩压
	uint8_t diastolic;		//舒张压
}bp_calibra_t;

typedef struct
{
	SETTINGS_STATUS flag;
	bool temp_is_on;				//temp
	bool hr_is_on;					//heart rate
	bool bpt_is_on;					//blood pressure
	bool spo2_is_on;				//blood oxygen
	bool wake_screen_by_wrist;
	bool wrist_off_check;
	bool fall_check;		
	uint8_t location_type;	//1:only wifi,2:only gps,3:wifi+gps,4:gps+wifi
	uint16_t target_steps;
	uint32_t health_interval;
	phd_measure_t phd_infor;
	location_interval_t dot_interval;
	bp_calibra_t bp_calibra;
	alarm_infor_t alarm[ALARM_MAX];
}global_settings_t;

extern bool need_save_time;
extern bool need_save_settings;
extern bool need_reset_settings;

extern uint8_t g_fw_version[64];

extern global_settings_t global_settings;
extern RESET_STATUS g_reset_status;

extern void InitSystemSettings(void);
extern void SaveSystemSettings(void);
extern void ResetFactoryDefault(void);
extern void ResetLocalData(void);
extern void ResetHealthData(void);
extern void ResetSportData(void);
#endif/*__SETTINGS_H__*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/nvs.h>
#include <dk_buttons_and_leds.h>
#include "settings.h"
#include "datetime.h"
#ifdef CONFIG_ALARM_SUPPORT
#include "alarm.h"
#endif
#include "codetrans.h"
#include "inner_flash.h"
#include "logger.h"

bool need_save_settings = false;
bool need_save_time = false;
bool need_reset_settings = false;

uint8_t g_fw_version[64] = VERSION_STR;

RESET_STATUS g_reset_status = RESET_STATUS_IDLE;

static bool reset_start_flag = false;
static bool reset_reboot_flag = false;

global_settings_t global_settings = {0};

static void FactoryResetCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(reset_timer, FactoryResetCallBack, NULL);
static void FactoryResetStartCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(reset_start_timer, FactoryResetStartCallBack, NULL);

extern sys_date_timer_t date_time;

const sys_date_timer_t FACTORY_DEFAULT_TIME = 
{
	2023,
	1,
	1,
	0,
	0,
	0,
	0		//0=sunday
};

const global_settings_t FACTORY_DEFAULT_SETTINGS = 
{
	SETTINGS_STATUS_NORMAL,	//status flag
	true,					//temp turn on
	true,					//heart rate turn on
	true,					//blood pressure turn on
	true,					//blood oxygen turn on		
	true,					//wake screen by wrist
	false,					//wrist off check
	true,					//fall check
	3,						//location type: 1:only wifi,2:only gps,3:wifi+gps,4:gps+wifi
	0,						//target steps
	60,						//health interval
	{true,1},				//PHD
	{500,60},				//position interval
	{120,75},				//pb calibration
	{						//alarm
		{false,0,0,0},		
		{false,0,0,0},
		{false,0,0,0},
		{false,0,0,0},
		{false,0,0,0},
		{false,0,0,0},
		{false,0,0,0},
		{false,0,0,0},
	},
};

void FactoryResetCallBack(struct k_timer *timer_id)
{
	switch(g_reset_status)
	{
	case RESET_STATUS_IDLE:
		break;

	case RESET_STATUS_RUNNING:
		g_reset_status = RESET_STATUS_FAIL;
		break;

	case RESET_STATUS_SUCCESS:
		reset_reboot_flag = true;
		break;

	case RESET_STATUS_FAIL:
		g_reset_status = RESET_STATUS_IDLE;
		break;
	}
}

void FactoryResetStartCallBack(struct k_timer *timer_id)
{
	reset_start_flag = true;
}

void SaveSystemDateTime(void)
{
	SaveDateTimeToInnerFlash(date_time);
}

void ResetSystemTime(void)
{
	memcpy(&date_time, &FACTORY_DEFAULT_TIME, sizeof(sys_date_timer_t));
	SaveSystemDateTime();
}

void InitSystemDateTime(void)
{
	sys_date_timer_t mytime = {0};

	ReadDateTimeFromInnerFlash(&mytime);
	
	if(!CheckSystemDateTimeIsValid(mytime))
	{
		memcpy(&mytime, &FACTORY_DEFAULT_TIME, sizeof(sys_date_timer_t));
	}
	memcpy(&date_time, &mytime, sizeof(sys_date_timer_t));

	SaveSystemDateTime();
	StartSystemDateTime();
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void SaveFactoryTestResults(FT_STATUS type, void *ret)
{
	SaveFtResultsToInnerFlash(type, ret);
}

void ResetFactoryTestResults(void)
{
	memset(&ft_smt_results, 0, sizeof(ft_smt_results_t));
	SaveFactoryTestResults(FT_STATUS_SMT, &ft_smt_results);

	memset(&ft_assem_results, 0, sizeof(ft_assem_results_t));
	SaveFactoryTestResults(FT_STATUS_ASSEM, &ft_assem_results);

	g_ft_status = FT_STATUS_SMT;
	SaveFtStatusToInnerFlash(g_ft_status);
}

void InitFactoryTestResults(void)
{
	ReadFtStatusFromInnerFlash(&g_ft_status);
	ReadFtResultsFromInnerFlash(FT_STATUS_SMT, &ft_smt_results);
	ReadFtResultsFromInnerFlash(FT_STATUS_ASSEM, &ft_assem_results);
	
	if(FactorySmtTestFinished())
	{
		g_ft_status = FT_STATUS_ASSEM;
		SaveFtStatusToInnerFlash(g_ft_status);
	}
}
#endif

void SaveSystemSettings(void)
{
	SaveSettingsToInnerFlash(global_settings);
}

void ResetSystemSettings(void)
{
	memcpy(&global_settings, &FACTORY_DEFAULT_SETTINGS, sizeof(global_settings_t));
	SaveSystemSettings();
}

void InitSystemSettings(void)
{
	int err;

	ReadSettingsFromInnerFlash(&global_settings);

	switch(global_settings.flag)
	{
	case SETTINGS_STATUS_INIT:
		ResetInnerFlash();
		memcpy(&global_settings, &FACTORY_DEFAULT_SETTINGS, sizeof(global_settings_t));
		SaveSystemSettings();
		break;

	case SETTINGS_STATUS_OTA:
		memcpy(&global_settings, &FACTORY_DEFAULT_SETTINGS, sizeof(global_settings_t));
		SaveSystemSettings();
		break;
		
	case SETTINGS_STATUS_NORMAL:
		break;		
	}

	InitSystemDateTime();

#ifdef CONFIG_FACTORY_TEST_SUPPORT
	InitFactoryTestResults();
#endif

#ifdef CONFIG_ALARM_SUPPORT	
	AlarmRemindInit();
#endif
	mmi_chset_init();
}

void ResetLocalData(void)
{
	clear_cur_local_in_record();
	clear_local_in_record();
}

void ResetHealthData(void)
{
#if defined(CONFIG_PPG_SUPPORT)||defined(CONFIG_TEMP_SUPPORT)
	clear_cur_health_in_record();
	clear_health_in_record();
#endif

#ifdef CONFIG_PPG_SUPPORT
	//ClearAllHrRecData();
	//ClearAllSpo2RecData();
	//ClearAllBptRecData();
	sh_clear_bpt_cal_data();
#endif

#ifdef CONFIG_TEMP_SUPPORT
	//ClearAllTempRecData();
#endif
}

void ResetSportData(void)
{
#if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
	clear_cur_sport_in_record();
	clear_sport_in_record();
#endif

#ifdef CONFIG_IMU_SUPPORT
#ifdef CONFIG_STEP_SUPPORT
	ClearAllStepRecData();
#endif

#ifdef CONFIG_SLEEP_SUPPORT
	ClearAllSleepRecData();
#endif
#endif
}

void ResetFactoryDefault(void)
{
#ifdef CONFIG_FACTORY_TEST_SUPPORT
	ResetFactoryTestResults();	
#endif

	ResetSystemTime();
	ResetSystemSettings();

	ResetLocalData();
	ResetHealthData();
	ResetSportData();

	if(k_timer_remaining_get(&reset_timer) > 0)
		k_timer_stop(&reset_timer);
	
	g_reset_status = RESET_STATUS_SUCCESS;
}

void SettingsMsgPorcess(void)
{
	if(need_save_time)
	{
		SaveSystemDateTime();
		need_save_time = false;
	}
	
	if(need_save_settings)
	{
		need_save_settings = false;				
		SaveSystemSettings();
		//SendSettingsData();
	}

	if(need_reset_settings)
	{
		need_reset_settings = false;
		k_timer_start(&reset_timer, K_MSEC(1000), K_NO_WAIT);
		ResetFactoryDefault();
	}
	if(reset_start_flag)
	{
		reset_start_flag = false;
		k_timer_start(&reset_timer, K_MSEC(1000), K_NO_WAIT);
		ResetFactoryDefault();
	}
	if(reset_reboot_flag)
	{
		reset_reboot_flag = false;
		sys_reboot(0);
	}
}

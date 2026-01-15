/****************************************Copyright (c)************************************************
** File Name:			    max32674.c
** Descriptions:			PPG process source file
** Created By:				xie biao
** Created Date:			2021-05-19
** Modified Date:      		2021-05-19 
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/random/rand32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <nrfx.h>
#include "uart.h"
#include "Max32674.h"
#include "max_sh_interface.h"
#include "max_sh_api.h"
#include "inner_flash.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#include "logger.h"

#define PPG_DEBUG

#define PPG_HR_COUNT_MAX		10
#define PPG_HR_DEL_MIN_NUM		6
#define PPG_SPO2_COUNT_MAX		3
#define PPG_SPO2_DEL_MIN_NUM	1
#define PPG_SCC_COUNT_MAX		5
#define NOTIFY_TIMER_INTERVAL	5

bool ppg_int_event = false;
bool ppg_bpt_is_calbraed = false;
bool ppg_bpt_cal_need_update = false;
bool ppg_skin_contacted_flag = false;

sys_date_timer_t g_health_check_time = {0};
PPG_WORK_STATUS g_ppg_status = PPG_STATUS_PREPARE;

static bool ppg_appmode_init_flag = false;

static bool ppg_delay_start_flag = false;
static bool ppg_start_flag = false;
static bool ppg_test_flag = false;
static bool ppg_stop_flag = false;
static bool ppg_get_data_flag = false;
static bool ppg_get_cal_flag = false;
static bool ppg_stop_cal_flag = false;
static bool menu_start_hr = false;
static bool menu_start_spo2 = false;
static bool menu_start_bpt = false;
#ifdef CONFIG_FACTORY_TEST_SUPPORT
static bool ft_start_hr = false;
#endif

uint8_t ppg_test_info[256] = {0};

uint8_t ppg_power_flag = 0;	//0:关闭 1:正在启动 2:启动成功
static uint8_t whoamI=0, rst=0;

uint8_t g_ppg_trigger = 0;
uint8_t g_ppg_data = PPG_DATA_MAX;
uint8_t g_ppg_alg_mode = ALG_MODE_HR_SPO2;
uint8_t g_ppg_bpt_status = BPT_STATUS_GET_EST;
uint8_t g_ppg_ver[64] = {0};

uint8_t g_hr = 0;
uint8_t g_spo2 = 0;
bpt_data g_bpt = {0};

static uint8_t scc_check_sum = 0;
static uint8_t SCC_COMPARE_MAX = PPG_SCC_COUNT_MAX;
static void ppg_set_appmode_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ppg_appmode_timer, ppg_set_appmode_timerout, NULL);
static void ppg_get_data_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ppg_get_hr_timer, ppg_get_data_timerout, NULL);
static void ppg_bpt_est_start_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ppg_bpt_est_start_timer, ppg_bpt_est_start_timerout, NULL);
static void ppg_skin_check_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ppg_skin_check_timer, ppg_skin_check_timerout, NULL);


void GetPPGData(uint8_t *hr, uint8_t *spo2, uint8_t *systolic, uint8_t *diastolic)
{
	if(hr != NULL)
		*hr = g_hr;
	
	if(spo2 != NULL)
		*spo2 = g_spo2;
	
	if(systolic != NULL)
		*systolic = g_bpt.systolic;
	
	if(diastolic != NULL)
		*diastolic = g_bpt.diastolic;
}

bool PPGIsSccCheck(void)
{
	if((g_ppg_trigger&TRIGGER_BY_SCC) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool PPGIsWorking(void)
{
	if(ppg_power_flag == 0)
		return false;
	else
		return true;
}

void ppg_get_data_timerout(struct k_timer *timer_id)
{
	ppg_get_data_flag = true;
}

static void ppg_set_appmode_timerout(struct k_timer *timer_id)
{
	ppg_appmode_init_flag = true;
}

bool PPGSenSorSet(void)
{
	int status = -1;
	uint8_t hubMode = 0x00;
	uint8_t period = 0;

	status = sh_get_sensorhub_operating_mode(&hubMode);
	if((hubMode != 0x00) && (status != SS_SUCCESS))
	{
	#ifdef PPG_DEBUG
		LOGD("work mode is not app mode:%d", hubMode);
	#endif
		return false;
	}

	if(g_ppg_alg_mode == ALG_MODE_BPT)
	{
		if(!ppg_bpt_is_calbraed)
		{
			if(!ppg_bpt_cal_need_update)
			{
			#ifdef PPG_DEBUG
				LOGD("check bpt cal success");
			#endif
				sh_set_bpt_cal_data();
				g_ppg_bpt_status = BPT_STATUS_GET_EST;
			}
			else
			{
			#ifdef PPG_DEBUG
				LOGD("check bpt cal fail, req cal from algo, (%d/%d)", global_settings.bp_calibra.systolic, global_settings.bp_calibra.diastolic);
			#endif
				sh_req_bpt_cal_data();
				g_ppg_bpt_status = BPT_STATUS_GET_CAL;

				k_timer_start(&ppg_get_hr_timer, K_MSEC(200), K_MSEC(200));
				return;
			}
		}
		else
		{
			g_ppg_bpt_status = BPT_STATUS_GET_EST;
		}
		
		//Enable AEC
		sh_set_cfg_wearablesuite_afeenable(1);
		//Enable automatic calculation of target PD current
		sh_set_cfg_wearablesuite_autopdcurrentenable(1);
		//Set minimum PD current to 12.5uA
		sh_set_cfg_wearablesuite_minpdcurrent(125);
		//Set initial PD current to 31.2uA
		sh_set_cfg_wearablesuite_initialpdcurrent(312);
		//Set target PD current to 31.2uA
		sh_set_cfg_wearablesuite_targetpdcurrent(312);
		//Enable SCD
		sh_set_cfg_wearablesuite_scdenable(1);
		//Set the output format to Sample Counter byte, Sensor Data and Algorithm
		sh_set_data_type(SS_DATATYPE_BOTH, true);
		//set fifo thresh
		sh_set_fifo_thresh(1);
		//Set the samples report period to 40ms(minimum is 32ms for BPT).
		sh_set_report_period(25);
		//Enable the sensor.
		sensorhub_enable_sensors();
		//set algo mode
		sh_set_cfg_wearablesuite_algomode(0x0);
		//set algo oper mode
		sensorhub_set_algo_operation_mode(SH_OPERATION_WHRM_BPT_MODE);
		g_algo_sensor_stat.bpt_algo_enabled = 1;
		//set algo submode
		sensorhub_set_algo_submode(SH_OPERATION_WHRM_BPT_MODE, SH_BPT_MODE_ESTIMATION);
		//enable algo
		sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE_OS6X, SENSORHUB_MODE_BASIC);
		g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1 = 1;

		k_timer_start(&ppg_get_hr_timer, K_MSEC(200), K_MSEC(500));
	}
	else if(g_ppg_alg_mode == ALG_MODE_HR_SPO2)
	{
		//set fifo thresh
		sh_set_fifo_thresh(1);
		//Set the samples report period to 40ms(minimum is 32ms for BPT).
		sh_set_report_period(25);
		//Enable AEC
		sh_set_cfg_wearablesuite_afeenable(1);
		//Enable automatic calculation of target PD current
		sh_set_cfg_wearablesuite_autopdcurrentenable(1);
		//Set the output format to Sample Counter byte, Sensor Data and Algorithm
		sh_set_data_type(SS_DATATYPE_BOTH, true);
		//Enable the sensor.
		sensorhub_enable_sensors();
		//set algo oper mode
		sensorhub_set_algo_operation_mode(SH_OPERATION_WHRM_MODE);
		g_algo_sensor_stat.bpt_algo_enabled = 0;
		//Set the WAS algorithm operation mode to Continuous HRM + Continuous SpO2
		sh_set_cfg_wearablesuite_algomode(0x0);
		//enable algo
		sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE_OS6X, SENSORHUB_MODE_BASIC);
		g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1 = 1;

		k_timer_start(&ppg_get_hr_timer, K_MSEC(1*1000), K_MSEC(1*1000));
	}
	
	return true;
}

void StartSensorhubCallBack(void)
{
	bool ret = false;
	
	ret = PPGSenSorSet();
	if(ret)
	{
		if(ppg_power_flag == 0)
		{
		#ifdef PPG_DEBUG
			LOGD("ppg hr has been stop!");
		#endif
			k_timer_stop(&ppg_get_hr_timer);
			return;
		}
	#ifdef PPG_DEBUG	
		LOGD("ppg hr start success!");
	#endif
	
		MapcsSendData(UART_DATA_PPG, COM_PPG_SET_OPEN, sizeof(COM_PPG_SET_OPEN));
		ppg_power_flag = 2;
	}
	else
	{
	#ifdef PPG_DEBUG
		LOGD("ppg hr start false!");
	#endif
		MapcsSendData(UART_DATA_PPG, COM_PPG_SET_CLOSE, sizeof(COM_PPG_SET_CLOSE));
		ppg_power_flag = 0;
	}
}

void StartSensorhub(void)
{
	SH_set_to_APP_mode();
	k_timer_start(&ppg_appmode_timer, K_MSEC(500), K_NO_WAIT);
}

static void ppg_bpt_est_start_timerout(struct k_timer *timer_id)
{
#ifdef PPG_DEBUG	
	LOGD("begin");
#endif

	g_ppg_alg_mode = ALG_MODE_BPT;
	g_ppg_data = PPG_DATA_BPT;
	g_ppg_bpt_status = BPT_STATUS_GET_EST;

	ppg_start_flag = true;
}

void PPGRestartToBpt(void)
{
	k_timer_start(&ppg_bpt_est_start_timer, K_MSEC(500), K_NO_WAIT);
}

void PPGGetSensorHubData(void)
{
	int ret = 0;
	int num_bytes_to_read = 0;
	uint8_t hubStatus = 0;
	uint8_t databuf[READ_SAMPLE_COUNT_MAX*SS_NORMAL_BPT_PACKAGE_SIZE] = {0};
	whrm_wspo2_suite_sensorhub_data sensorhub_out = {0};
	bpt_sensorhub_data bpt = {0};
	accel_data accel = {0};
	max86176_data max86176 = {0};

	ret = sh_get_sensorhub_status(&hubStatus);
#ifdef PPG_DEBUG	
	LOGD("ret:%d, hubStatus:%d", ret, hubStatus);
#endif
	if(hubStatus & SS_MASK_STATUS_FIFO_OUT_OVR)
	{
	#ifdef PPG_DEBUG
		LOGD("SS_MASK_STATUS_FIFO_OUT_OVR");
	#endif
	}

	if((0 == ret) && (hubStatus & SS_MASK_STATUS_DATA_RDY))
	{
		int u32_sampleCnt = 0;

	#ifdef PPG_DEBUG
		LOGD("FIFO ready");
	#endif

		num_bytes_to_read += SS_PACKET_COUNTERSIZE;
		if(g_algo_sensor_stat.max86176_enabled)
			num_bytes_to_read += SSMAX86176_MODE1_DATASIZE;
		if(g_algo_sensor_stat.accel_enabled)
			num_bytes_to_read += SSACCEL_MODE1_DATASIZE;
		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1)
			num_bytes_to_read += SSWHRM_WSPO2_SUITE_MODE1_DATASIZE;
		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode2)
			num_bytes_to_read += SSWHRM_WSPO2_SUITE_MODE2_DATASIZE;
		if(g_algo_sensor_stat.bpt_algo_enabled)
			num_bytes_to_read += SSBPT_ALGO_DATASIZE;
		if(g_algo_sensor_stat.algo_raw_enabled)
			num_bytes_to_read += SSRAW_ALGO_DATASIZE;

		ret = sensorhub_get_output_sample_number(&u32_sampleCnt);
		if(ret == SS_SUCCESS)
		{
		#ifdef PPG_DEBUG
			LOGD("sample count is:%d", u32_sampleCnt);
		#endif
		}
		else
		{
		#ifdef PPG_DEBUG
			LOGD("read sample count fail:%d", ret);
		#endif
		}

		WAIT_MS(5);

		if(u32_sampleCnt > READ_SAMPLE_COUNT_MAX)
			u32_sampleCnt = READ_SAMPLE_COUNT_MAX;
		
		ret = sh_read_fifo_data(u32_sampleCnt, num_bytes_to_read, databuf, sizeof(databuf));
		if(ret == SS_SUCCESS)
		{
			uint16_t heart_rate=0,blood_oxy=0;
			uint32_t i,j,index = 0;
			uint8_t scd_status = 0;
			static uint8_t count=0;
			uint8_t buffer[CAL_RESULT_SIZE+16] = {0};
			uint32_t len;

			for(i=0,j=0;i<u32_sampleCnt;i++)
			{
				index = i * num_bytes_to_read + 1;

				if(g_ppg_alg_mode == ALG_MODE_BPT)
				{
					strcpy(buffer, COM_PPG_GET_DATA);
					len = strlen(COM_PPG_GET_DATA);
					memcpy(&buffer[len], (void*)&databuf[index], SS_NORMAL_BPT_PACKAGE_SIZE);
					MapcsSendData(UART_DATA_PPG, buffer, SS_NORMAL_BPT_PACKAGE_SIZE+len);
					
					bpt_algo_data_rx(&bpt, &databuf[index+SS_PACKET_COUNTERSIZE + SSMAX86176_MODE1_DATASIZE + SSACCEL_MODE1_DATASIZE + SSWHRM_WSPO2_SUITE_MODE1_DATASIZE]);
					if(g_ppg_bpt_status == BPT_STATUS_GET_CAL)
					{
						if((bpt.status == 2) && (bpt.perc_comp == 100))
						{
						#ifdef PPG_DEBUG
							LOGD("get calbration data success!");
						#endif
							sh_get_bpt_cal_data();
						
							strcpy(buffer, COM_PPG_SAVE_CAL);
							len = strlen(COM_PPG_SAVE_CAL);
							memcpy(&buffer[len], sh_bpt_cal, CAL_RESULT_SIZE);
							MapcsSendData(UART_DATA_PPG, buffer, CAL_RESULT_SIZE+len);
							
							ppg_bpt_cal_need_update = false;
							ppg_stop_cal_flag = true;
							PPGRestartToBpt();
						}
					}
				}
				else if(g_ppg_alg_mode == ALG_MODE_HR_SPO2)
				{
					strcpy(buffer, COM_PPG_GET_DATA);
					len = strlen(COM_PPG_GET_DATA);
					memcpy(&buffer[len], (void*)&databuf[index], SS_NORMAL_PACKAGE_SIZE);
					MapcsSendData(UART_DATA_PPG, buffer, SS_NORMAL_PACKAGE_SIZE+len);
				}
			}
		}
		else
		{
		#ifdef PPG_DEBUG
			LOGD("read FIFO result fail:%d", ret);
		#endif
		}
	}
	else
	{
	#ifdef PPG_DEBUG
		LOGD("FIFO status is not ready:%d,%d", ret, hubStatus);
	#endif
	}
}

void ppg_delay_start_timerout(struct k_timer *timer_id)
{
	ppg_delay_start_flag = true;
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartPPG(void)
{
	ft_start_hr = true;
}

void FTStopPPG(void)
{
	ppg_stop_flag = true;
}
#endif

void StartSCC(void)
{
	if(PPGIsWorking())
		return;

	g_ppg_trigger |= TRIGGER_BY_SCC;
	g_ppg_data = PPG_DATA_HR;
	g_ppg_alg_mode = ALG_MODE_HR_SPO2;

	ppg_skin_contacted_flag = true;
    ppg_start_flag = true;
	k_timer_start(&ppg_skin_check_timer, K_SECONDS(10), K_NO_WAIT);
}

bool CheckSCC(void)
{
	return ppg_skin_contacted_flag;
}

void StartPPG(PPG_DATA_TYPE data_type, PPG_TRIGGER_SOURCE trigger_type)
{
#ifdef PPG_DEBUG	
	LOGD("data:%d, type:%d", data_type, trigger_type);
#endif

	if(PPGIsSccCheck())
		PPGStopCheck();

	switch(trigger_type)
	{
	case TRIGGER_BY_HOURLY:
		break;
		
	case TRIGGER_BY_MENU:
		break;

#ifdef CONFIG_BLE_SUPPORT
	case TRIGGER_BY_APP_ONE_KEY:
		if(0)//(!is_wearing())
		{
			//MCU_send_app_one_key_measure_data();
			return;
		}
		if(PPGIsWorking())
		{
			if(g_ppg_data == PPG_DATA_HR)
				g_ppg_trigger |= trigger_type;
			//else
			//	MCU_send_app_one_key_measure_data();

			return;
		}
		break;
		
	case TRIGGER_BY_APP:
		if(0)//(!is_wearing())
		{
			uint8_t hr = 0;
			
			//MCU_send_app_get_ppg_data(data_type, &hr);
			return;
		}
		if(PPGIsWorking())
		{
			if(g_ppg_data == PPG_DATA_HR)
				g_ppg_trigger |= trigger_type;
			//else
			//	MCU_send_app_get_ppg_data(data_type, &g_hr);

			return;
		}
		break;
#endif

	case TRIGGER_BY_FT:
		g_ppg_trigger = TRIGGER_BY_FT;
		g_ppg_alg_mode = ALG_MODE_HR_SPO2;
		g_ppg_data = PPG_DATA_HR;
		ppg_start_flag = true;
		return;
		
	default:
		return;
	}

	g_ppg_trigger |= trigger_type;
	g_ppg_data = data_type;
	switch(data_type)
	{
	case PPG_DATA_HR:
		g_ppg_alg_mode = ALG_MODE_HR_SPO2;
		g_hr = 0;
		break;
		
	case PPG_DATA_SPO2:
		g_ppg_alg_mode = ALG_MODE_HR_SPO2;
		g_spo2 = 0;
		break;
		
	case PPG_DATA_BPT:
		g_ppg_alg_mode = ALG_MODE_BPT;
		memset(&g_bpt, 0, sizeof(bpt_data));
		break;
	}

	ppg_start_flag = true;
}

void MenuStartHr(void)
{
	menu_start_hr = true;
}

void MenuStopHr(void)
{
	ppg_stop_flag = true;
}

void MenuStartSpo2(void)
{
	menu_start_spo2 = true;
}

void MenuStopSpo2(void)
{
	ppg_stop_flag = true;
}

void MenuStartBpt(void)
{
	menu_start_bpt = true;
}

void MenuStopBpt(void)
{
	ppg_stop_flag = true;
}

void MenuStartPPG(void)
{
	g_ppg_trigger |= TRIGGER_BY_MENU;
	ppg_start_flag = true;
}

void MenuStopPPG(void)
{
	g_ppg_trigger = 0;
	g_ppg_alg_mode = ALG_MODE_HR_SPO2;
	g_ppg_bpt_status = BPT_STATUS_GET_EST;
	ppg_stop_flag = true;
}

void PPGStartCheck(void)
{
#ifdef PPG_DEBUG
	LOGD("ppg_power_flag:%d", ppg_power_flag);
#endif
	if(ppg_power_flag > 0)
		return;

	if(g_ppg_alg_mode == ALG_MODE_BPT)
		SCC_COMPARE_MAX = 3*PPG_SCC_COUNT_MAX;
	else
		SCC_COMPARE_MAX = PPG_SCC_COUNT_MAX;
	scc_check_sum = SCC_COMPARE_MAX;
	
	PPG_Enable();
	
	ppg_power_flag = 1;

	StartSensorhub();
}

void PPGStopCheck(void)
{
#ifdef PPG_DEBUG
	LOGD("ppg_power_flag:%d", ppg_power_flag);
#endif
	k_timer_stop(&ppg_appmode_timer);
	k_timer_stop(&ppg_get_hr_timer);
	k_timer_stop(&ppg_bpt_est_start_timer);
	k_timer_stop(&ppg_skin_check_timer);

	if(ppg_power_flag == 0)
		return;

	sensorhub_disable_sensor();
	sensorhub_disable_algo();

	PPG_Disable();

	ppg_power_flag = 0;

#ifdef CONFIG_BLE_SUPPORT
	if((g_ppg_trigger&TRIGGER_BY_APP_ONE_KEY) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_APP_ONE_KEY);

	if((g_ppg_trigger&TRIGGER_BY_APP) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_APP);
#endif	

	if((g_ppg_trigger&TRIGGER_BY_MENU) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_MENU);

	if((g_ppg_trigger&TRIGGER_BY_HOURLY) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_HOURLY);

	if((g_ppg_trigger&TRIGGER_BY_SCC) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_SCC);

#ifdef CONFIG_FACTORY_TEST_SUPPORT
	if((g_ppg_trigger&TRIGGER_BY_FT) != 0)
		g_ppg_trigger = g_ppg_trigger&(~TRIGGER_BY_FT);
#endif
}

void PPGStopBptCal(void)
{
	k_timer_stop(&ppg_get_hr_timer);
	
	sensorhub_disable_sensor();
	sensorhub_disable_algo();

	PPG_Disable();

	ppg_power_flag = 0;
}

static void ppg_skin_check_timerout(struct k_timer *timer_id)
{
	if(g_ppg_trigger == TRIGGER_BY_SCC)
		ppg_stop_flag = true;
}

void UartPPGEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;

	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, PPG_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(PPG_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_PPG_SET_OPEN)) != NULL)
		{
			ptr1 += strlen(COM_PPG_SET_OPEN);
			if((ptr2 = strstr(ptr1, COM_PPG_GET_HR)) != NULL)
			{
				StartPPG(PPG_DATA_HR, TRIGGER_BY_MENU);
			}
			else if((ptr2 = strstr(ptr1, COM_PPG_GET_SPO2)) != NULL)
			{
				StartPPG(PPG_DATA_SPO2, TRIGGER_BY_MENU);
			}
			else if((ptr2 = strstr(ptr1, COM_PPG_GET_BPT)) != NULL)
			{
				ptr2 += strlen(COM_PPG_GET_BPT);
				if((ptr1 = strstr(ptr2, COM_PPG_GET_CAL)) != NULL)
				{
					ppg_bpt_cal_need_update = true;					
				}
				else
				{
					memcpy(sh_bpt_cal, ptr2, data_len-(ptr2-data));
					ppg_bpt_cal_need_update = false;
				}
				
				StartPPG(PPG_DATA_BPT, TRIGGER_BY_MENU);				
			}
		}
		else if((ptr1 = strstr(ptr, COM_PPG_SET_CLOSE)) != NULL)
		{
			PPGStopCheck();
		}
		else if((ptr1 = strstr(ptr, COM_PPG_UPGRADE)) != NULL)
		{
			SH_OTA_upgrade_start();
		}
		else if((ptr1 = strstr(ptr, COM_PPG_UPGRADE_PAGE_NUM)) != NULL)
		{
			ptr1 += strlen(COM_PPG_UPGRADE_PAGE_NUM);
			SH_OTA_upgrade_set_page_num(ptr1, data_len-(ptr1-data));
		}
		else if((ptr1 = strstr(ptr, COM_PPG_UPGRADE_VECTOR_BYTES)) != NULL)
		{
			ptr1 += strlen(COM_PPG_UPGRADE_VECTOR_BYTES);
			SH_OTA_upgrade_set_vector_bytes(ptr1, data_len-(ptr1-data));
		}
		else if((ptr1 = strstr(ptr, COM_PPG_UPGRADE_AUTH_BYTES)) != NULL)
		{
			ptr1 += strlen(COM_PPG_UPGRADE_AUTH_BYTES);
			SH_OTA_upgrade_set_auth_bytes(ptr1, data_len-(ptr1-data));
		}
		else if((ptr1 = strstr(ptr, COM_PPG_UPGRADE_FLASH_PAGE)) != NULL)
		{
			ptr1 += strlen(COM_PPG_UPGRADE_FLASH_PAGE);
		    SH_OTA_upgrade_set_flash_pages(ptr1, data_len-(ptr1-data));
		}
	}
}

void PPG_init(void)
{
#ifdef PPG_DEBUG
	LOGD("PPG_init");
#endif

	if(!sh_init_interface())
		return;

#ifdef PPG_DEBUG	
	LOGD("PPG_init done!");
#endif
}

void PPGMsgProcess(void)
{
	if(ppg_int_event)
	{
		ppg_int_event = false;
	}
	
#ifdef CONFIG_FACTORY_TEST_SUPPORT
	if(ft_start_hr)
	{
		StartPPG(PPG_DATA_HR, TRIGGER_BY_FT);
		ft_start_hr = false;
	}
#endif

	if(menu_start_hr)
	{
		StartPPG(PPG_DATA_HR, TRIGGER_BY_MENU);
		menu_start_hr = false;
	}
	
	if(menu_start_spo2)
	{
		StartPPG(PPG_DATA_SPO2, TRIGGER_BY_MENU);
		menu_start_spo2 = false;
	}
	
	if(menu_start_bpt)
	{
		StartPPG(PPG_DATA_BPT, TRIGGER_BY_MENU);
		menu_start_bpt = false;
	}
	
	if(ppg_start_flag)
	{
	#ifdef PPG_DEBUG
		LOGD("PPG start!");
	#endif
		PPGStartCheck();
		ppg_start_flag = false;
	}
	
	if(ppg_stop_flag)
	{
	#ifdef PPG_DEBUG
		LOGD("PPG stop!");
	#endif
		PPGStopCheck();
		ppg_stop_flag = false;
	}

	if(ppg_stop_cal_flag)
	{
	#ifdef PPG_DEBUG	
		LOGD("bpt cal stop");
	#endif
		PPGStopBptCal();
		ppg_stop_cal_flag = false;
	}
	
	if(ppg_get_cal_flag)
	{
		ppg_get_cal_flag = false;
	}
	
	if(ppg_get_data_flag)
	{
		PPGGetSensorHubData();
		ppg_get_data_flag = false;
	}
	
	if(ppg_delay_start_flag)
	{
		switch(g_ppg_data)
		{
		case PPG_DATA_HR:
			StartPPG(PPG_DATA_HR, TRIGGER_BY_HOURLY);
			break;

		case PPG_DATA_SPO2:
			StartPPG(PPG_DATA_SPO2, TRIGGER_BY_HOURLY);
			break;

		case PPG_DATA_BPT:
			StartPPG(PPG_DATA_BPT, TRIGGER_BY_HOURLY);
			break;
		}

		ppg_delay_start_flag = false;
	}

	if(ppg_appmode_init_flag)
	{
		StartSensorhubCallBack();
		ppg_appmode_init_flag = false;
	}
}

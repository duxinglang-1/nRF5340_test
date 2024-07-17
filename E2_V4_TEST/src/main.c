/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <power/reboot.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <dk_buttons_and_leds.h>
#include "datetime.h"
#include "inner_flash.h"
#include "uart_ble.h"
#include "settings.h"
#include "codetrans.h"
#ifdef CONFIG_WATCHDOG
#include "watchdog.h"
#endif
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif/*CONFIG_WIFI_SUPPORT*/
#include "logger.h"

static bool sys_pwron_completed_flag = false;

void system_init(void)
{
	InitSystemSettings();

	key_init();
#ifdef CONFIG_WIFI_SUPPORT
	wifi_init();
#endif
	ble_init();
}

void work_init(void)
{
	if(IS_ENABLED(CONFIG_WATCHDOG))
	{
		watchdog_init_and_start(&k_sys_work_q);
	}
}

bool system_is_completed(void)
{
	return sys_pwron_completed_flag;
}

void system_init_completed(void)
{
	if(!sys_pwron_completed_flag)
		sys_pwron_completed_flag = true;
}

/***************************************************************************
* 描  述 : main函数 
* 入  参 : 无 
* 返回值 : int 类型
**************************************************************************/
int main(void)
{
	LOGD("begin");

	work_init();
	system_init();
	
	while(1)
	{
		KeyMsgProcess();
		TimeMsgProcess();
		SettingsMsgPorcess();
	#ifdef CONFIG_WIFI_SUPPORT
		WifiMsgProcess();
	#endif
		UartMsgProc();
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FactoryTestProccess();
	#endif
		system_init_completed();
		k_cpu_idle();
	}
}

/****************************************Copyright (c)************************************************
** File Name:			    buzzer.c
** Descriptions:			buzzer function main source file
** Created By:				xie biao
** Created Date:			2024-04-30
** Modified Date:      		2024-04-30
** Version:			    	V1.0
******************************************************************************************************/
#include <nrfx.h>
#include <nrfx_gpiote.h>
#include <zephyr/drivers/pwm.h>
#include "buzzer.h"
	
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm0), okay)
#define BUZZER_DEVICE DT_NODELABEL(pwm0)
#else
//#error "pwm0 devicetree node is disabled"
#define BUZZER_DEVICE	""
#endif

#define PWM_FREQ		4000
#define PWM_DUTY1		1/2
#define PWM_DUTY2		1/4
#define PWM_DUTY3		1/8
#define PWM_DUTY4		1/16

struct device *pwm_buzzer;
struct pwm_dt_spec sBuzzer1,sBuzzer2,sBuzzer3,sBuzzer4;

void buzzer_set(bool OnOff)
{
	pwm_set_pulse_dt(&sBuzzer1, OnOff ? (sBuzzer1.period*PWM_DUTY1) : 0);
	//pwm_set_pulse_dt(&sBuzzer2, OnOff ? (sBuzzer2.period*PWM_DUTY2) : 0);
	//pwm_set_pulse_dt(&sBuzzer3, OnOff ? (sBuzzer3.period*PWM_DUTY3) : 0);
	//pwm_set_pulse_dt(&sBuzzer4, OnOff ? (sBuzzer4.period*PWM_DUTY4) : 0);
}

int buzzer_init(void)
{
	pwm_buzzer = DEVICE_DT_GET(BUZZER_DEVICE);
	if(!pwm_buzzer) 
	{
		return;
	}

	sBuzzer1.dev = pwm_buzzer;
	sBuzzer1.channel = 0;
	sBuzzer1.period = PWM_HZ(PWM_FREQ);
	sBuzzer1.flags = PWM_POLARITY_NORMAL;

	//sBuzzer2.dev = pwm_buzzer;
	//sBuzzer2.channel = 1;
	//sBuzzer2.period = PWM_HZ(PWM_FREQ);
	//sBuzzer2.flags = PWM_POLARITY_NORMAL;

	//sBuzzer3.dev = pwm_buzzer;
	//sBuzzer3.channel = 2;
	//sBuzzer3.period = PWM_HZ(PWM_FREQ);
	//sBuzzer3.flags = PWM_POLARITY_NORMAL;

	//sBuzzer4.dev = pwm_buzzer;
	//sBuzzer4.channel = 3;
	//sBuzzer4.period = PWM_HZ(PWM_FREQ);
	//sBuzzer4.flags = PWM_POLARITY_NORMAL;

	//buzzer_set(true);
}

void BuzzerMsgProcess(void)
{
	while(1)
	{
		buzzer_set(true);
		k_sleep(K_MSEC(200));
		buzzer_set(false);
		k_sleep(K_MSEC(50));
		buzzer_set(true);
		k_sleep(K_MSEC(200));
		buzzer_set(false);
		k_sleep(K_MSEC(500));
	}
}


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
#define BUZZER_1_DEVICE DT_NODELABEL(pwm0)
#else
//#error "pwm0 devicetree node is disabled"
#define BUZZER_1_DEVICE	""
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm1), okay)
#define BUZZER_2_DEVICE DT_NODELABEL(pwm1)
#else
//#error "pwm0 devicetree node is disabled"
#define BUZZER_2_DEVICE	""
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm2), okay)
#define BUZZER_3_DEVICE DT_NODELABEL(pwm2)
#else
//#error "pwm0 devicetree node is disabled"
#define BUZZER_3_DEVICE	""
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm3), okay)
#define BUZZER_4_DEVICE DT_NODELABEL(pwm3)
#else
//#error "pwm0 devicetree node is disabled"
#define BUZZER_4_DEVICE	""
#endif


#define PWM_FREQ_1		4000
#define PWM_FREQ_2		2700
#define PWM_FREQ_3		2000
#define PWM_FREQ_4		1000

#define PWM_DUTY1		1/2
#define PWM_DUTY2		1/2
#define PWM_DUTY3		1/2
#define PWM_DUTY4		1/2

struct device *pwm_buzzer1,*pwm_buzzer2,*pwm_buzzer3,*pwm_buzzer4;
struct pwm_dt_spec sBuzzer1,sBuzzer2,sBuzzer3,sBuzzer4;

void buzzer_set(bool OnOff)
{
	pwm_set_pulse_dt(&sBuzzer1, OnOff ? (sBuzzer1.period*PWM_DUTY1) : 0);
	pwm_set_pulse_dt(&sBuzzer2, OnOff ? (sBuzzer2.period*PWM_DUTY2) : 0);
	//pwm_set_pulse_dt(&sBuzzer3, OnOff ? (sBuzzer3.period*PWM_DUTY3) : 0);
	//pwm_set_pulse_dt(&sBuzzer4, OnOff ? (sBuzzer4.period*PWM_DUTY4) : 0);
}

int buzzer_init(void)
{
	pwm_buzzer1 = DEVICE_DT_GET(BUZZER_1_DEVICE);
	if(!pwm_buzzer1) 
	{
		return;
	}

	sBuzzer1.dev = pwm_buzzer1;
	sBuzzer1.channel = 0;
	sBuzzer1.period = PWM_HZ(PWM_FREQ_1);
	sBuzzer1.flags = PWM_POLARITY_NORMAL;

	pwm_buzzer2 = DEVICE_DT_GET(BUZZER_2_DEVICE);
	if(!pwm_buzzer2) 
	{
		return;
	}

	sBuzzer2.dev = pwm_buzzer2;
	sBuzzer2.channel = 0;
	sBuzzer2.period = PWM_HZ(PWM_FREQ_2);
	sBuzzer2.flags = PWM_POLARITY_NORMAL;

	//pwm_buzzer3 = DEVICE_DT_GET(BUZZER_3_DEVICE);
	//if(!pwm_buzzer3) 
	//{
	//	return;
	//}

	//sBuzzer3.dev = pwm_buzzer3;
	//sBuzzer3.channel = 0;
	//sBuzzer3.period = PWM_HZ(PWM_FREQ_3);
	//sBuzzer3.flags = PWM_POLARITY_NORMAL;

	//pwm_buzzer4 = DEVICE_DT_GET(BUZZER_4_DEVICE);
	//if(!pwm_buzzer4) 
	//{
	//	return;
	//}

	//sBuzzer4.dev = pwm_buzzer4;
	//sBuzzer4.channel = 0;
	//sBuzzer4.period = PWM_HZ(PWM_FREQ_4);
	//sBuzzer4.flags = PWM_POLARITY_NORMAL;

	buzzer_set(true);
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

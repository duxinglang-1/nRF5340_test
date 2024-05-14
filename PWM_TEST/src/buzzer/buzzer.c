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
uint8_t count = 0;
int32_t freq_offset[] = {
							-1730,-1730,-1730,		//5
							-1725,-1725,-1725,		//5
							-1720,-1720,-1720,		//5
							-1715,-1715,-1715,		//5
							-1710,-1710,-1710,		//10
							-1700,-1700,-1700,		//15
							-1685,-1685,-1685,		//15
							-1670,-1670,-1670,		//20
							-1650,-1650,-1650,		//20
							-1630,-1630,-1630,		//30
							-1600,-1600,-1600,		//50
							-1550,-1550,-1550,		//60
							-1490,-1490,-1490,		//70
							-1420,-1420,-1420,		//80
							-1340,-1340,-1340,		//100
							-1240,-1240,-1240,		//120
							-1120,-1120,-1120,		//150
							-970,-970,-970,			//200
							-770,-770,-770,			//150
							-620,-620,-620,			//120
							-500,-500,-500,			//100
							-400,-400,-400,			//80
							-320,-320,-320,			//70
							-250,-250,-250,			//60
							-190,-190,-190,			//50
							-140,-140,-140,			//30
							-110,-110,-110,			//20
							-90,-90,-90,			//20
							-70,-70,-70,			//15
							-55,-55,-55,			//15
							-40,-40,-40,			//10
							-30,-30,-30,			//10
							-20,-20,-20,			//5
							-15,-15,-15,			//5
							-10,-10,-10,			//5
							-5,-5,-5,-5,			//5
							 0,0,0,0,				//0
							-5,-5,-5,-5,			//5
							-10,-10,-10,			//5
							-15,-15,-15,			//5
							-20,-20,-20,			//5
							-30,-30,-30,			//10
							-40,-40,-40,			//10
							-55,-55,-55,			//15
							-70,-70,-70,			//15
							-90,-90,-90,			//20
							-110,-110,-110,			//20
							-140,-140,-140,			//30
							-190,-190,-190,			//50
							-250,-250,-250,			//60
							-320,-320,-320,			//70
							-400,-400,-400,			//80
							-500,-500,-500,			//100
							-620,-620,-620,			//120
							-770,-770,-770,			//150
							-970,-970,-970,			//200
							-1120,-1120,-1120,		//150
							-1240,-1240,-1240,		//120
							-1340,-1340,-1340,		//100
							-1420,-1420,-1420,		//80
							-1490,-1490,-1490,		//70
							-1550,-1550,-1550,		//60
							-1600,-1600,-1600,		//50
							-1630,-1630,-1630,		//30
							-1650,-1650,-1650,		//20
							-1670,-1670,-1670,		//20
							-1685,-1685,-1685,		//15
							-1700,-1700,-1700,		//15
							-1710,-1710,-1710,		//10
							-1715,-1715,-1715,		//5
							-1720,-1720,-1720,		//5
							-1725,-1725,-1725		//5
							//-1730,-1730,-1730,	    //5
						};

void buzzer_set(bool OnOff)
{
	if(OnOff)
	{
		count++;
		if(count == (sizeof(freq_offset))/(sizeof(freq_offset[0])))
			count = 0;
		sBuzzer1.period = PWM_HZ(PWM_FREQ_1+freq_offset[count]);
	}
	pwm_set_pulse_dt(&sBuzzer1, OnOff ? (sBuzzer1.period*PWM_DUTY1) : 0);
	//pwm_set_pulse_dt(&sBuzzer2, OnOff ? (sBuzzer2.period*PWM_DUTY2) : 0);
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
	sBuzzer1.period = PWM_HZ(PWM_FREQ_1+freq_offset[0]);
	sBuzzer1.flags = PWM_POLARITY_NORMAL;

	//pwm_buzzer2 = DEVICE_DT_GET(BUZZER_2_DEVICE);
	//if(!pwm_buzzer2) 
	//{
	//	return;
	//}

	//sBuzzer2.dev = pwm_buzzer2;
	//sBuzzer2.channel = 0;
	//sBuzzer2.period = PWM_HZ(PWM_FREQ_2);
	//sBuzzer2.flags = PWM_POLARITY_NORMAL;

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
		k_sleep(K_MSEC(5));
		//buzzer_set(false);
		//k_sleep(K_MSEC(50));
		//buzzer_set(true);
		//k_sleep(K_MSEC(200));
		//buzzer_set(false);
		//k_sleep(K_MSEC(500));
	}
}


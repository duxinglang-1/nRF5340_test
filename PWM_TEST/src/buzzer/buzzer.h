/****************************************Copyright (c)************************************************
** File Name:			    buzzer.h
** Descriptions:			buzzer function main head file
** Created By:				xie biao
** Created Date:			2024-04-30
** Modified Date:      		2024-04-30
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __BUZZER_H__
#define __BUZZER_H__
	
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/types.h>
	
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize the PWM LED.
 *
 *	@param None
 *
 *	@retval 0 if the operation was successful, otherwise a (negative) error code.
 */
int pwm_led_init(void);

/** @brief Set the PWM level.
 *
 *	@param desired_lvl Level value.
 */
void pwm_led_set(uint16_t desired_lvl);

#ifdef __cplusplus
}
#endif
	
#endif/*__BUZZER_H__*/

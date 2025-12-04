/****************************************Copyright (c)************************************************
** File Name:			    gxts04.h
** Descriptions:			gxts04 interface process source file
** Created By:				xie biao
** Created Date:			2021-12-24
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "temp.h"
#include "gxts04.h"
#include "logger.h"
#ifdef CONFIG_CRC_SUPPORT
#include "crc_check.h"
#endif

static struct device *i2c_temp;
static struct device *gpio_temp;

#ifdef CONFIG_CRC_SUPPORT
static CRC_8 crc_8_CUSTOM = {0x31,0xff,0x00,false,false};
#endif

static uint16_t HardwareID = 0;
static uint32_t measure_count = 0;
static float t_sensor = 0.0;		//传感器温度值
static float t_body = 0.0; 			//显示的温度值
static float t_predict = 0.0;		//预测的人体温度值
static float t_temp80 = 0.0;		//预测的人体温度值


static uint8_t init_i2c(void)
{
	i2c_temp = DEVICE_DT_GET(TEMP_DEV);
	if(!i2c_temp)
	{
		return -1;
	} 
	else
	{
		i2c_configure(i2c_temp, I2C_SPEED_SET(I2C_SPEED_FAST));
		return 0;
	}
}

static int32_t gxts04_read_data(uint16_t cmd, uint8_t* bufp, uint16_t len)
{
	uint8_t data[2] = {0};
	uint32_t rslt = 0;

	data[0] = (uint8_t)(0x00ff&(cmd>>8));
	data[1] = (uint8_t)(0x00ff&cmd);

	rslt = i2c_write(i2c_temp, data, 2, GXTS04_I2C_ADDR);
	if(rslt == 0)
	{
		if((cmd == CMD_MEASURE_LOW_POWER) || (cmd == CMD_MEASURE_NORMAL))
		{
			k_sleep(K_MSEC(20));
		}
		
		rslt = i2c_read(i2c_temp, bufp, len, GXTS04_I2C_ADDR);
	}

	return rslt;
}

static int32_t gxts04_write_data(uint16_t cmd)
{
	uint8_t data[2] = {0};
	uint32_t rslt = 0;

	data[0] = (uint8_t)(0x00ff&(cmd>>8));
	data[1] = (uint8_t)(0x00ff&cmd);

	rslt = i2c_write(i2c_temp, data, 2, GXTS04_I2C_ADDR);
	return rslt;
}

static void temp_sensor_init(void)
{
	uint8_t databuf[2] = {0};

	gxts04_write_data(CMD_RESET);
	k_usleep(200);

	gxts04_write_data(CMD_WAKEUP);
	gxts04_read_data(CMD_READ_ID, &databuf, 2);
	gxts04_write_data(CMD_SLEEP);

	HardwareID = databuf[0]*0x100 + databuf[1];
#ifdef TEMP_DEBUG	
	LOGD("temp id:%x", HardwareID);
#endif
}

bool gxts04_init(uint16_t *id)
{
	if(init_i2c() != 0)
		return;

	temp_sensor_init();
	*id = HardwareID;
	
	//if(HardwareID != GXTS04_ID)
	//	return false;
	//else
		return true;
}

void gxts04_start(void)
{
	temp_sensor_init();
	
	measure_count = 0;
	t_sensor = 0.0;
	t_body = 0.0;
	t_predict = 0.0;
	t_temp80 = 0.0;
}

void gxts04_stop(void)
{
	measure_count = 0;
}

void GetTemperature(uint8_t *sensor_temp)
{
	uint8_t databuf[10] = {0};

	gxts04_write_data(CMD_WAKEUP);
	gxts04_read_data(CMD_MEASURE_LOW_POWER, &databuf, 10);
	gxts04_write_data(CMD_SLEEP);

#ifdef TEMP_DEBUG
	LOGD("temp:%02x,%02x,%02x", databuf[0],databuf[1],databuf[2]);
#endif

	memcpy(sensor_temp, databuf, sizeof(databuf));
}

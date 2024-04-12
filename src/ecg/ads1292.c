/****************************************Copyright (c)************************************************
** File Name:			    ads1292.c
** Descriptions:			Sensor source file for ADS1292
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "ads1292.h"
#include "logger.h"

#define ADS_DEBUG

struct device *spi_ecg;
struct device *gpio_ecg;
static struct gpio_callback gpio_cb;

static struct spi_buf_set tx_bufs,rx_bufs;
static struct spi_buf tx_buff,rx_buff;
static struct spi_config spi_cfg;
static struct spi_cs_control spi_cs_ctr;

static uint8_t spi_tx_buf[8] = {0};  
static uint8_t spi_rx_buf[8] = {0};  

static bool ecg_trige_flag = false;

void EcgInterruptHandle(void)
{
	ecg_trige_flag = true;
}

void ECG_CS_LOW(void)
{
	gpio_pin_set(gpio_ecg, ECG_CS_PIN, 0);
}

void ECG_CS_HIGH(void)
{
	gpio_pin_set(gpio_ecg, ECG_CS_PIN, 1);
}

void ads_sys_set(ADS_SYS_COMMAND sys_command)
{
	int err;
	
	spi_tx_buf[0] = sys_command;

	tx_buff.buf = spi_tx_buf;
	tx_buff.len = 1;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	ECG_CS_LOW();
	
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error: %d", err);
	#endif
	}

	ECG_CS_HIGH();
}

void ads_data_write(ADS_DATA_COMMAND data_command)
{
	int err;
		
	spi_tx_buf[0] = data_command;

	tx_buff.buf = spi_tx_buf;
	tx_buff.len = 1;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	ECG_CS_LOW();
	
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error: %d", err);
	#endif
	}

	ECG_CS_HIGH();

}

void ads_data_read(ADS_DATA_COMMAND data_command, uint8_t *rxbuf, uint32_t len)
{
	int err;
	
	spi_tx_buf[0] = data_command;

	tx_buff.buf = spi_tx_buf;
	tx_buff.len = 1;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	ECG_CS_LOW();
	
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 001: %d", err);
	#endif
	}

	rx_buff.buf = rxbuf;
	rx_buff.len = len;
	rx_bufs.buffers = &rx_buff;
	rx_bufs.count = 1;

	err = spi_transceive(spi_ecg, &spi_cfg, NULL, &rx_bufs);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 002: %d", err);
	#endif
	}

	ECG_CS_HIGH();
}

void ads_reg_write(uint8_t addr, uint8_t *write_buf, uint32_t write_len)
{
	int err;
	
	spi_tx_buf[0] = ADS_RED_COM_WREG|(addr&0x1f);
	spi_tx_buf[1] = 0x00|(write_len-1);

	tx_buff.buf = spi_tx_buf;
	tx_buff.len = 2;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	ECG_CS_LOW();
	
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 001: %d", err);
	#endif
	}

	tx_buff.buf = write_buf;
	tx_buff.len = write_len;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 002: %d", err);
	#endif
	}

	ECG_CS_HIGH();
}

void ads_reg_read(uint8_t addr, uint8_t *read_buf, uint32_t read_len)
{
	int err;
	
	spi_tx_buf[0] = ADS_REG_COM_RREG|(addr&0x1f);
	spi_tx_buf[1] = 0x00|(read_len-1);

	tx_buff.buf = spi_tx_buf;
	tx_buff.len = 2;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	ECG_CS_LOW();
	
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, NULL);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 001: %d", err);
	#endif
	}

	rx_buff.buf = read_buf;
	rx_buff.len = read_len;
	rx_bufs.buffers = &rx_buff;
	rx_bufs.count = 1;

	err = spi_transceive(spi_ecg, &spi_cfg, NULL, &rx_bufs);
	if(err)
	{
	#ifdef ADS_DEBUG
		LOGD("SPI error 002: %d", err);
	#endif
	}

	ECG_CS_HIGH();
}

void ECG_SPI_Init(void)
{
	spi_ecg = DEVICE_DT_GET(ECG_DEVICE);
	if(!spi_ecg) 
	{
	#ifdef FLASH_DEBUG
		LOGD("Could not get %s device", ECG_DEVICE);
	#endif
		return;
	}

	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8);
	spi_cfg.frequency = 1000000;
	spi_cfg.slave = 0;
}

void ads1292_sensor_init(void)
{
	uint8_t device_id;

	ads_data_write(ADS_DATA_COM_SDATAC);
	ads_sys_set(ADS_SYS_COM_STOP);
	
	ads_reg_read(ADS_REG_ID, &device_id, 1);
	LOGD("id:%x", device_id);
	switch((device_id&0xE0)>>5)
	{
	case 0b000:
		LOGD("Reserved 000");
		break;
	case 0b001:
		LOGD("Reserved 001");
		break;
	case 0b010:
		LOGD("ADS1x9x device");
		break;
	case 0b011:
		LOGD("ADS1292R device");
		break;
	case 0b100:
		LOGD("Reserved 100");
		break;
	case 0b101:
		LOGD("Reserved 101");
		break;
	case 0b110:
		LOGD("Reserved 110");
		break;
	case 0b111:
		LOGD("Reserved 111");
		break;
	}

	switch(device_id&0x03)
	{
	case 0b00:
		LOGD("ADS1191");
		break;
	case 0b01:
		LOGD("ADS1192");
		break;
	case 0b10:
		LOGD("ADS1291");
		break;
	case 0b11:
		LOGD("ADS1292 and ADS1292R");
		break;
	}
}

void ads1292_init(void)
{
	int err;
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_UP;

  	//¶Ë¿Ú³õÊ¼»¯
  	gpio_ecg = DEVICE_DT_GET(ECG_PORT);
	if(!gpio_ecg)
	{
		return;
	}

	gpio_pin_configure(gpio_ecg, ECG_RESET_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_ecg, ECG_RESET_PIN, 0);
	k_sleep(K_MSEC(2000));
	gpio_pin_set(gpio_ecg, ECG_RESET_PIN, 1);
	gpio_pin_configure(gpio_ecg, ECG_START_PIN, GPIO_OUTPUT);
	gpio_pin_configure(gpio_ecg, ECG_CS_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_ecg, ECG_CS_PIN, 1);
	
	//ecg ready interrupt
	gpio_pin_configure(gpio_ecg, ECG_DRDY_PIN, flag);
	gpio_pin_interrupt_configure(gpio_ecg, ECG_DRDY_PIN, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, EcgInterruptHandle, BIT(ECG_DRDY_PIN));
	gpio_add_callback(gpio_ecg, &gpio_cb);
	gpio_pin_interrupt_configure(gpio_ecg, ECG_DRDY_PIN, GPIO_INT_ENABLE|GPIO_INT_EDGE_FALLING);
	
	ECG_SPI_Init();

	ads1292_sensor_init();
}

void asd1292_msg_process(void)
{
	if(ecg_trige_flag)
	{
		ecg_trige_flag = false;
	}
}

/****************************************Copyright (c)************************************************
** File Name:			    LCD_EQTAC175T1371_CO5300.c
** Descriptions:			The LCD_EQTAC175T1371_CO5300 screen drive source file
** Created By:				xie biao
** Created Date:			2025-05-22
** Modified Date:      		2025-05-22
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/drivers/spi.h>
#include <nrfx_qspi.h>
#include <zephyr/drivers/gpio.h>
#include "lcd.h"
#include "font.h"
#include "settings.h"
#include "logger.h"
#ifdef LCD_BACKLIGHT_CONTROLED_BY_PMU
#include "Max20353.h"
#endif
#ifdef LCD_EQTAC175T1371_CO5300
#include "LCD_EQTAC175T1371_CO5300.h"

#define SPI_BUF_LEN	8
#define SPI_MUIT_BY_CS

#define QSPI_STD_CMD_WRSR  0x01
#define QSPI_STD_CMD_RSTEN 0x66
#define QSPI_STD_CMD_RST   0x99

struct device *qspi_lcd;
struct device *spi_lcd;
struct device *gpio_lcd;

static struct k_timer backlight_timer;

static const nrf_qspi_cinstr_len_t cmd_cinstr_len[] = 
{
	NRF_QSPI_CINSTR_LEN_4B,
	NRF_QSPI_CINSTR_LEN_5B,
	NRF_QSPI_CINSTR_LEN_6B,
	NRF_QSPI_CINSTR_LEN_7B,
	NRF_QSPI_CINSTR_LEN_8B,
	NRF_QSPI_CINSTR_LEN_9B
};

struct spi_buf_set tx_bufs,rx_bufs;
struct spi_buf tx_buff,rx_buff;

static struct spi_config spi_cfg = 
{
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.frequency = 8000000,
	.slave = 0,
#ifndef SPI_MUIT_BY_CS	
	.cs = {
			.gpio = GPIO_DT_SPEC_GET(LCD_DEV, cs_gpios),
			.delay = 0u,
			},
#endif			
};

static LCD_BL_MODE bl_mode = LCD_BL_AUTO;

static uint8_t tx_buffer[SPI_BUF_LEN] = {0};
static uint8_t rx_buffer[SPI_BUF_LEN] = {0};

uint8_t lcd_data_buffer[2*LCD_DATA_LEN] = {0};	//xb add 20200702 a pix has 2 byte data

#ifdef SPI_MUIT_BY_CS
void LCD_CS_LOW(void)
{
	gpio_pin_set(gpio_lcd, CS, 0);
}

void LCD_CS_HIGH(void)
{
	gpio_pin_set(gpio_lcd, CS, 1);
}
#endif

#ifdef LCD_TYPE_QSPI
static void LCD_QSPI_handler(nrfx_qspi_evt_t event, void *p_context)
{
	if (event == NRFX_QSPI_EVENT_DONE)
	{
		LOGD("QSPI done");
		//gc9c01_complete(dev_data);
	}
}

static void LCD_QSPI_Init(void)
{
	nrfx_qspi_config_t QSPIconfig;

	//qspi_lcd = DEVICE_DT_GET(LCD_DEV);
	//if(!qspi_lcd)
	//	return;
	
	/* Configure XIP offset */
	QSPIconfig.xip_offset = 0;
	QSPIconfig.skip_gpio_cfg=false;
	QSPIconfig.skip_psel_cfg=false;

	/* Configure pins */
	QSPIconfig.pins.sck_pin = 17;
	QSPIconfig.pins.csn_pin = 18;
	QSPIconfig.pins.io0_pin = 13;
	QSPIconfig.pins.io1_pin = 14;
	QSPIconfig.pins.io2_pin = 15;
	QSPIconfig.pins.io3_pin = 16;

	QSPIconfig.prot_if.readoc = NRF_QSPI_READOC_FASTREAD;
	QSPIconfig.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4O;
	QSPIconfig.prot_if.addrmode = NRF_QSPI_ADDRMODE_24BIT;
	QSPIconfig.prot_if.dpmconfig = false;

	/* Configure physical interface */
 	QSPIconfig.phy_if.sck_freq = NRF_QSPI_FREQ_DIV16;
	QSPIconfig.phy_if.sck_delay = 0x05;
	QSPIconfig.phy_if.spi_mode = NRF_QSPI_MODE_0;
	QSPIconfig.phy_if.dpmen = false;

	nrfx_err_t err_code = nrfx_qspi_init(&QSPIconfig, LCD_QSPI_handler, NULL);
	if(err_code != NRFX_SUCCESS)
	{
		LOGD("QSPI initialization failed with error: %d\n", err_code);
		return;
	}
	else
	{
		LOGD("QSPI initialized successfully");
	}
}

static void LCD_QSPI_Transceive(uint8_t *tx_buff, uint32_t tx_len, uint8_t *rx_buff)
{
	nrfx_err_t res;
	nrf_qspi_cinstr_conf_t cfg = { 
									.opcode    = 0x02,
									.io2_level = false,
									.io3_level = true,
									.wipwait   = false,
									.wren 	   = false
								};

	res = nrfx_qspi_cinstr_xfer(&cfg, tx_buff, NULL);
	if(res == NRFX_SUCCESS)
	{
		LOGD("QSPI send success!");
	}
	else
	{
		LOGD("QSPI send fail! res:%d", res);
	}
}
#endif

static void LCD_SPI_Init(void)
{
	spi_lcd = DEVICE_DT_GET(LCD_DEV);
	if(!spi_lcd) 
	{
		return;
	}

	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	spi_cfg.frequency = 8000000;
	spi_cfg.slave = 0;

#ifndef SPI_MUIT_BY_CS
	spi_cfg.cs.gpio = GPIO_DT_SPEC_GET(LCD_DEV, cs_gpios);
	spi_cfg.cs.delay = 0U;
#endif
}

static void LCD_SPI_Transceive(uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen)
{
	int err;
	
	tx_buff.buf = txbuf;
	tx_buff.len = txbuflen;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;

	rx_buff.buf = rxbuf;
	rx_buff.len = rxbuflen;
	rx_bufs.buffers = &rx_buff;
	rx_bufs.count = 1;

#ifdef SPI_MUIT_BY_CS
	LCD_CS_LOW();
#endif
	err = spi_transceive(spi_lcd, &spi_cfg, &tx_bufs, &rx_bufs);
#ifdef SPI_MUIT_BY_CS
	LCD_CS_HIGH();
#endif
	if(err)
	{
	}
}

//LCD��ʱ����
void Delay(unsigned int dly)
{
	k_sleep(K_MSEC(dly));
}

static void backlight_timer_handler(struct k_timer *timer)
{
	lcd_sleep_in = true;
}

//���ݽӿں���
//i:8λ����
void Write_Data(uint8_t i) 
{	
	lcd_data_buffer[0] = i;
	
	LCD_SPI_Transceive(lcd_data_buffer, 1, NULL, 0);
}

//----------------------------------------------------------------------
//д�Ĵ�������
//i:�Ĵ���ֵ
void WriteComm(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
	uint32_t len = 4;
	uint8_t tx_buff[8],rx_buff[8];
	nrfx_err_t res;
	nrf_qspi_cinstr_conf_t cfg = { 
									.opcode    = 0x02,
									.io2_level = true,
									.io3_level = true,
									.wipwait   = false,
									.wren	   = false
								};

	gpio_pin_set(gpio_lcd, RS, 0);

	tx_buff[0] = 0x00;
	tx_buff[1] = cmd;
	tx_buff[2] = 0x00;

#ifdef LCD_TYPE_QSPI
	if(data_len < ARRAY_SIZE(cmd_cinstr_len))
	{
		cfg.length = cmd_cinstr_len[data_len];

		memcpy(&tx_buff[3], data, data_len);
		res = nrfx_qspi_cinstr_xfer(&cfg, tx_buff, rx_buff);
		if(res == NRFX_SUCCESS)
		{
			LOGD("nrfx_qspi_cinstr_xfer success!");
		}
		else
		{
			LOGD("nrfx_qspi_cinstr_xfer fail!res:%d", res);
		}
	}
	else
	{
		cfg.length = NRF_QSPI_CINSTR_LEN_1B;
		size_t len_1;

		res = nrfx_qspi_lfm_start(&cfg);
		if(res == NRFX_SUCCESS)
		{
			LOGD("nrfx_qspi_lfm_start success!");
		}
		else
		{
			LOGD("nrfx_qspi_lfm_start fail!res:%d", res);
		}

		len_1 = sizeof(tx_buff) - 3;
		memcpy(&tx_buff[3], data, len_1);

		if(res == NRFX_SUCCESS)
		{
			res = nrfx_qspi_lfm_xfer(tx_buff, NULL, sizeof(tx_buff), false);
			if(res == NRFX_SUCCESS)
			{
				LOGD("nrfx_qspi_lfm_xfer 001 success!");
			}
			else
			{
				LOGD("nrfx_qspi_lfm_xfer 001 fail!res:%d", res);
			}
		}

		if(res == NRFX_SUCCESS)
		{
			res = nrfx_qspi_lfm_xfer(data + len_1, NULL, data_len - len_1, true);
			if(res == NRFX_SUCCESS)
			{
				LOGD("nrfx_qspi_lfm_xfer 002 success!");
			}
			else
			{
				LOGD("nrfx_qspi_lfm_xfer 002 fail!res:%d", res);
			}
		}
	}
	
#elif defined(LCD_TYPE_SPI)
	if(data != NULL && data_len > 0)
	{
		memcpy(&tx_buff[4], data, data_len);
		len += data_len;
	}

	LCD_SPI_Transceive(tx_buff, len, NULL, 0);
#endif

	gpio_pin_set(gpio_lcd, RS, 1);
}

//дLCD����
//i:Ҫд����?
void WriteData(uint8_t i)
{
	gpio_pin_set(gpio_lcd, RS, 1);
	Write_Data(i);
	gpio_pin_set(gpio_lcd, RS, 0);
}

void WriteDispData(uint8_t DataH,uint8_t DataL)
{
	gpio_pin_set(gpio_lcd, RS, 1);

	lcd_data_buffer[0] = DataH;
	lcd_data_buffer[1] = DataL;
	
	LCD_SPI_Transceive(lcd_data_buffer, 2, NULL, 0);
}

//LCD���㺯��
//color:Ҫ������ɫ
void WriteOneDot(unsigned int color)
{ 
	WriteDispData(color>>8, color);
}

////////////////////////////////////////////////���Ժ���//////////////////////////////////////////
void BlockWrite(unsigned int x,unsigned int y,unsigned int w,unsigned int h) //reentrant
{
	uint8_t buffer[8];
	
	x += 0;
	y += 0;

	buffer[0] = x>>8;
	buffer[1] = x;
	buffer[2] = (x+w-1)>>8;
	buffer[3] = (x+w-1);
	WriteComm(0x2A, buffer, 4);             
          
  	buffer[0] = y>>8;
  	buffer[1] = y;
  	buffer[2] = (y+h-1)>>8;
	buffer[3] = (y+h-1);
	WriteComm(0x2B, buffer, 4);             

	WriteComm(0x2c, NULL, 0);
}

void DispColor(uint32_t total, uint16_t color)
{
	uint32_t i,remain;      

	gpio_pin_set(gpio_lcd, RS, 1);
	
	while(1)
	{
		if(total <= LCD_DATA_LEN)
			remain = total;
		else
			remain = LCD_DATA_LEN;
		
		for(i=0;i<remain;i++)
		{
			lcd_data_buffer[2*i] = color>>8;
			lcd_data_buffer[2*i+1] = color;
		}
		
		LCD_SPI_Transceive(lcd_data_buffer, 2*remain, NULL, 0);

		if(remain == total)
			break;

		total -= remain;
	}
}

void DispData(uint32_t total, uint8_t *data)
{
	uint32_t i,remain;      

	gpio_pin_set(gpio_lcd, RS, 1);
	
	while(1)
	{
		if(total <= 2*LCD_DATA_LEN)
			remain = total;
		else
			remain = 2*LCD_DATA_LEN;
		
		for(i=0;i<remain;i++)
		{
			lcd_data_buffer[i] = data[i];
		}
		
		LCD_SPI_Transceive(lcd_data_buffer, remain, NULL, 0);

		if(remain == total)
			break;

		total -= remain;
	}
}

//���Ժ�������ʾRGB���ƣ�
void DispBand(void)	 
{
	uint32_t i;
	unsigned int color[8]={0xf800,0x07e0,0x001f,0xFFE0,0XBC40,0X8430,0x0000,0xffff};//0x94B2

	for(i=0;i<8;i++)
	{
		DispColor(COL*(ROW/8), color[i]);
	}

	DispColor(COL*(ROW%8), color[7]);
}

//���Ժ��������߿�
void DispFrame(void)
{
	unsigned int i,j;

	BlockWrite(0,0,COL,ROW);

	WriteDispData(0xf8, 0x00);

	for(i=0;i<COL-2;i++)
	{
		WriteDispData(0xFF, 0xFF);
	}
	
	WriteDispData(0x00, 0x1F);

	for(j=0;j<ROW-2;j++)
	{
		WriteDispData(0xf8, 0x00);
 
		for(i=0;i<COL-2;i++)
		{
			WriteDispData(0x00, 0x00);
		}

		WriteDispData(0x00, 0x1f);
	}

	WriteDispData(0xf8, 0x00);
 
	for(i=0;i<COL-2;i++)
	{
		WriteDispData(0xff, 0xff);
	}

	WriteDispData(0x00, 0x1f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool LCD_CheckID(void)
{
	WriteComm(0x04, NULL, 0);
	Delay(10); 

	if(rx_buffer[0] == 0x89 && rx_buffer[1] == 0xF0)
		return true;
	else
		return false;
}

//��������
//color:Ҫ����������?
void LCD_Clear(uint16_t color)
{
	BlockWrite(0,0,COL,ROW);//��λ

	DispColor(COL*ROW, color);
} 

//�����?
void LCD_BL_On(void)
{
#ifdef LCD_BACKLIGHT_CONTROLED_BY_PMU
	Set_Screen_Backlight_On();
#else
	gpio_pin_set(gpio_lcd, LEDA, 1);																											 
#endif	
}

//����ر�?
void LCD_BL_Off(void)
{
#ifdef LCD_BACKLIGHT_CONTROLED_BY_PMU
	Set_Screen_Backlight_Off();
#else
	gpio_pin_set(gpio_lcd, LEDA, 0);
#endif
}

//��Ļ˯��
void LCD_SleepIn(void)
{
	if(lcd_is_sleeping)
		return;

	WriteComm(0x28, NULL, 0);	
	WriteComm(0x10, NULL, 0);  		//Sleep in	
	Delay(120);             //��ʱ120ms

	lcd_is_sleeping = true;
}

//��Ļ����
void LCD_SleepOut(void)
{
	uint16_t bk_time;

	if(k_timer_remaining_get(&backlight_timer) > 0)
		k_timer_stop(&backlight_timer);

	if(global_settings.backlight_time != 0)
	{
		bk_time = global_settings.backlight_time;
		//xb add 2020-12-31 ̧������5����Զ�Ϣ��?
		if(sleep_out_by_wrist)
		{
			sleep_out_by_wrist = false;
			bk_time = 5;
		}

		switch(bl_mode)
		{
		case LCD_BL_ALWAYS_ON:
		case LCD_BL_OFF:
			break;

		case LCD_BL_AUTO:
			k_timer_start(&backlight_timer, K_SECONDS(bk_time), K_NO_WAIT);
			break;
		}
	}

	if(!lcd_is_sleeping)
		return;
	
	WriteComm(0x11, NULL, 0);  		//Sleep out	
	Delay(120);             //��ʱ120ms
	WriteComm(0x29, NULL, 0);

	lcd_is_sleeping = false;
}

//��Ļ���ñ�����ʱ
void LCD_ResetBL_Timer(void)
{
	if((bl_mode == LCD_BL_ALWAYS_ON) || (bl_mode == LCD_BL_OFF))
		return;
	
	if(k_timer_remaining_get(&backlight_timer) > 0)
		k_timer_stop(&backlight_timer);
	
	if(global_settings.backlight_time != 0)
		k_timer_start(&backlight_timer, K_SECONDS(global_settings.backlight_time), K_NO_WAIT);
}

//��ȡ��Ļ��ǰ����ģʽ
LCD_BL_MODE LCD_Get_BL_Mode(void)
{
	return bl_mode;
}

//��Ļ����ģʽ����
void LCD_Set_BL_Mode(LCD_BL_MODE mode)
{
	if(bl_mode == mode)
		return;
	
	switch(mode)
	{
	case LCD_BL_ALWAYS_ON:
		k_timer_stop(&backlight_timer);
		if(lcd_is_sleeping)
			LCD_SleepOut();
		break;

	case LCD_BL_AUTO:
		k_timer_stop(&backlight_timer);
		if(global_settings.backlight_time != 0)
			k_timer_start(&backlight_timer, K_SECONDS(global_settings.backlight_time), K_NO_WAIT);
		break;

	case LCD_BL_OFF:
		k_timer_stop(&backlight_timer);
		LCD_BL_Off();
		if(!lcd_is_sleeping)
			LCD_SleepIn();
		break;
	}

	bl_mode = mode;
}

//LCD��ʼ������
void LCD_Init(void)
{
	int err;
	uint8_t buffer[8] = {0};
	
  	//�˿ڳ�ʼ��
  	gpio_lcd = DEVICE_DT_GET(LCD_PORT);
	if(!gpio_lcd)
	{
		return;
	}

#ifdef SPI_MUIT_BY_CS
	gpio_pin_configure(gpio_lcd, CS, GPIO_OUTPUT);
#endif
	gpio_pin_configure(gpio_lcd, RST, GPIO_OUTPUT);
	gpio_pin_configure(gpio_lcd, RS, GPIO_OUTPUT);
	gpio_pin_configure(gpio_lcd, EN, GPIO_OUTPUT);

#ifdef LCD_TYPE_QSPI
	LCD_QSPI_Init();
#elif defined(LCD_TYPE_SPI)
	LCD_SPI_Init();
#endif

	gpio_pin_set(gpio_lcd, EN, 1);

	gpio_pin_set(gpio_lcd, RST, 1);
	Delay(10);
	gpio_pin_set(gpio_lcd, RST, 0);
	Delay(10);
	gpio_pin_set(gpio_lcd, RST, 1);
	Delay(120);

#if 1 //xb test 2025.05.22
	buffer[0] = 0x00;
	WriteComm(0xFE, buffer, 1);

	buffer[0] = 0x80;
	WriteComm(0xC4, buffer, 1);//SPI setting, mipi remove

	buffer[0] = 0x55;
	WriteComm(0x3A, buffer, 1);//55 RGB565, 77 RGB888  	

	buffer[0] = 0x00;
	WriteComm(0x35, buffer, 1);

	buffer[0] = 0x20;
	WriteComm(0x53, buffer, 1);

	buffer[0] = 0xFF;
	WriteComm(0x51, buffer, 1);

	buffer[0] = 0xFF;
	WriteComm(0x63, buffer, 1);

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x85;
	WriteComm(0x2A, buffer, 4);

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0xC1;
	WriteComm(0x2B, buffer, 4);

	WriteComm(0x11, NULL, 0);
	Delay(120);
	WriteComm(0x29, NULL, 0);

#else
	WriteComm(0xFE);
	WriteComm(0xEF); 

	WriteComm(0xEB);	
	WriteData(0x14); 

	WriteComm(0x84);			
	WriteData(0x40); 

	WriteComm(0x88);			
	WriteData(0x0A);

	WriteComm(0x89);			
	WriteData(0x21); 

	WriteComm(0x8A);			
	WriteData(0x00); 

	WriteComm(0x8B);			
	WriteData(0x80); 

	WriteComm(0x8C);			
	WriteData(0x01); 

	WriteComm(0x8D);			
	WriteData(0x01); 

	WriteComm(0xB6);			
	WriteData(0x00); 
	WriteData(0x00);

	WriteComm(0x36);
#ifdef LCD_SHOW_ROTATE_180	
	WriteData(0x88);
#else
	WriteData(0x48);
#endif

	WriteComm(0x3A);			
	WriteData(0x05); 

	WriteComm(0x90);			
	WriteData(0x08);
	WriteData(0x08);
	WriteData(0x08);
	WriteData(0x08); 

	WriteComm(0xBD);			
	WriteData(0x06);

	WriteComm(0xBC);			
	WriteData(0x00);	

	WriteComm(0xFF);			
	WriteData(0x60);
	WriteData(0x01);
	WriteData(0x04);

	WriteComm(0xC3);			
	WriteData(0x2F);
	WriteComm(0xC4);			
	WriteData(0x2F);

	WriteComm(0xC9);			
	WriteData(0x22);

	WriteComm(0xBE);			
	WriteData(0x11); 

	WriteComm(0xE1);			
	WriteData(0x10);
	WriteData(0x0E);

	WriteComm(0xDF);			
	WriteData(0x21);
	WriteData(0x0c);
	WriteData(0x02);

	WriteComm(0xF0);   
	WriteData(0x45);
	WriteData(0x09);
	WriteData(0x08);
	WriteData(0x08);
	WriteData(0x26);
	WriteData(0x2A);

	WriteComm(0xF1);	
	WriteData(0x43);
	WriteData(0x70);
	WriteData(0x72);
	WriteData(0x36);
	WriteData(0x37);  
	WriteData(0x6F);

	WriteComm(0xF2);   
	WriteData(0x45);
	WriteData(0x09);
	WriteData(0x08);
	WriteData(0x08);
	WriteData(0x26);
	WriteData(0x2A);

	WriteComm(0xF3);   
	WriteData(0x43);
	WriteData(0x70);
	WriteData(0x72);
	WriteData(0x36);
	WriteData(0x37); 
	WriteData(0x6F);

	WriteComm(0xED);	
	WriteData(0x1B); 
	WriteData(0x0B); 

	WriteComm(0xAE);			
	WriteData(0x77);

	WriteComm(0xCD);			
	WriteData(0x63);		

	WriteComm(0x70);			
	WriteData(0x07);
	WriteData(0x07);
	WriteData(0x04);
	WriteData(0x06); 
	WriteData(0x0F); 
	WriteData(0x09);
	WriteData(0x07);
	WriteData(0x08);
	WriteData(0x03);

	WriteComm(0xE8);			
	WriteData(0x34); 

	WriteComm(0x62);			
	WriteData(0x18);
	WriteData(0x0D);
	WriteData(0x71);
	WriteData(0xED);
	WriteData(0x70); 
	WriteData(0x70);
	WriteData(0x18);
	WriteData(0x0F);
	WriteData(0x71);
	WriteData(0xEF);
	WriteData(0x70); 
	WriteData(0x70);

	WriteComm(0x63);			
	WriteData(0x18);
	WriteData(0x11);
	WriteData(0x71);
	WriteData(0xF1);
	WriteData(0x70); 
	WriteData(0x70);
	WriteData(0x18);
	WriteData(0x13);
	WriteData(0x71);
	WriteData(0xF3);
	WriteData(0x70); 
	WriteData(0x70);

	WriteComm(0x64);			
	WriteData(0x28);
	WriteData(0x29);
	WriteData(0xF1);
	WriteData(0x01);
	WriteData(0xF1);
	WriteData(0x00);
	WriteData(0x07);

	WriteComm(0x66);			
	WriteData(0x3C);
	WriteData(0x00);
	WriteData(0xCD);
	WriteData(0x67);
	WriteData(0x45);
	WriteData(0x45);
	WriteData(0x10);
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x00);

	WriteComm(0x67);			
	WriteData(0x00);
	WriteData(0x3C);
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x01);
	WriteData(0x54);
	WriteData(0x10);
	WriteData(0x32);
	WriteData(0x98);

	WriteComm(0x74);			
	WriteData(0x10);	
	WriteData(0x85);	
	WriteData(0x80);
	WriteData(0x00); 
	WriteData(0x00); 
	WriteData(0x4E);
	WriteData(0x00);					

	WriteComm(0x98);			
	WriteData(0x3e);
	WriteData(0x07);

	WriteComm(0x35);	
	WriteComm(0x21);
	Delay(10);

	WriteComm(0x11);
	Delay(120);
	WriteComm(0x29);
	Delay(20);
	WriteComm(0x2C);	

	LCD_Clear(BLACK);		//����Ϊ��ɫ
	Delay(30);

	//��������
#ifdef LCD_BACKLIGHT_CONTROLED_BY_PMU
	Set_Screen_Backlight_On();
#else
	//gpio_pin_set(gpio_lcd, LEDK, 0);
	gpio_pin_set(gpio_lcd, LEDA, 1);
#endif
#endif

	lcd_is_sleeping = false;

	k_timer_init(&backlight_timer, backlight_timer_handler, NULL);

	if(global_settings.backlight_time != 0)
		k_timer_start(&backlight_timer, K_SECONDS(global_settings.backlight_time), K_NO_WAIT);	
}

#endif

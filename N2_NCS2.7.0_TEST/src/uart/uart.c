/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <stdio.h>
#include <string.h>
#include "logger.h"
#include "transfer_cache.h"
#include "datetime.h"
#include "Settings.h"
#include "Uart.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#include "inner_flash.h"
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif

//#define UART_DEBUG

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#define MAPCS_DEV DT_NODELABEL(uart0)
#else
#error "uart0 devicetree node is disabled"
#define MAPCS_DEV	""
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
#define MAPCS_PORT DT_NODELABEL(gpio0)
#else
#error "gpio0 devicetree node is disabled"
#define MAPCS_PORT	""
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
#define WIFI_DEV DT_NODELABEL(uart1)
#else
#error "uart1 devicetree node is disabled"
#define WIFI_DEV	""
#endif

#define MAPCS_INT_PIN		3
#define MAPCS_WAKE_PIN		8

#define BUF_MAXSIZE	4096

#ifdef CONFIG_PM_DEVICE
bool uart_mapcs_sleep_flag = false;
bool uart_mapcs_wake_flag = false;
bool uart_mapcs_is_waked = true;
#define UART_MAPCS_WAKE_HOLD_TIME_SEC		(5)

#ifdef CONFIG_WIFI_SUPPORT
bool uart_wifi_sleep_flag = false;
bool uart_wifi_wake_flag = false;
bool uart_wifi_is_waked = true;
#define UART_WIFI_WAKE_HOLD_TIME_SEC		(5)
#endif/*CONFIG_WIFI_SUPPORT*/
#endif

static bool uart_mapcs_send_data_flag = false;
static bool uart_mapcs_rece_data_flag = false;
static bool uart_mapcs_rece_frame_flag = false;

static CacheInfo uart_mapcs_send_cache = {0};
static CacheInfo uart_mapcs_rece_cache = {0};

static uint32_t uart_mapcs_rece_len=0;
static uint32_t uart_mapcs_send_len=0;

static uint8_t uart_mapcs_rx_buf[BUF_MAXSIZE]={0};

static K_FIFO_DEFINE(fifo_uart_mapcs_tx_data);
static K_FIFO_DEFINE(fifo_uart_mapcs_rx_data);

static struct device *uart_mapcs = NULL;
static struct device *gpio_mapcs = NULL;
static struct gpio_callback gpio_cb;

#ifdef CONFIG_WIFI_SUPPORT
static bool uart_wifi_send_data_flag = false;
static bool uart_wifi_rece_data_flag = false;
static bool uart_wifi_rece_frame_flag = false;

static CacheInfo uart_wifi_send_cache = {0};
static CacheInfo uart_wifi_rece_cache = {0};

static uint32_t uart_wifi_rece_len=0;
static uint32_t uart_wifi_send_len=0;

static uint8_t uart_wifi_rx_buf[BUF_MAXSIZE]={0};

static K_FIFO_DEFINE(fifo_uart_wifi_tx_data);
static K_FIFO_DEFINE(fifo_uart_wifi_rx_data);

static struct device *uart_wifi = NULL;
#endif/*CONFIG_WIFI_SUPPORT*/


static struct uart_data_t
{
	void  *fifo_reserved;
	uint8_t    data[BUF_MAXSIZE];
	uint16_t   len;
};

static void UartMapcsSendDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_mapcs_send_data_timer, UartMapcsSendDataCallBack, NULL);
static void UartMapcsReceDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_mapcs_rece_data_timer, UartMapcsReceDataCallBack, NULL);
static void UartMapcsReceFrameCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_mapcs_rece_frame_timer, UartMapcsReceFrameCallBack, NULL);
#ifdef CONFIG_PM_DEVICE
static void UartMapcsSleepInCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_mapcs_sleep_in_timer, UartMapcsSleepInCallBack, NULL);
#endif

#ifdef CONFIG_WIFI_SUPPORT
static void UartWifiSendDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_send_data_timer, UartWifiSendDataCallBack, NULL);
static void UartWifiReceDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_rece_data_timer, UartWifiReceDataCallBack, NULL);
static void UartWifiReceFrameCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_rece_frame_timer, UartWifiReceFrameCallBack, NULL);
#ifdef CONFIG_PM_DEVICE
static void UartWifiSleepInCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_sleep_in_timer, UartWifiSleepInCallBack, NULL);
#endif
#endif/*CONFIG_WIFI_SUPPORT*/

void copcs_wakeup_MCU(void)
{
	gpio_pin_set(gpio_mapcs, MAPCS_WAKE_PIN, 0);
	k_sleep(K_MSEC(5));
	gpio_pin_set(gpio_mapcs, MAPCS_WAKE_PIN, 1);
}

static void uart_send_data_handle(struct device *dev, uint8_t *buffer, uint32_t datalen)
{
	if(dev == uart_mapcs)
		copcs_wakeup_MCU();

#ifdef CONFIG_PM_DEVICE
	uart_sleep_out(dev);
#endif

#ifdef UART_DEBUG
	if(dev == uart_mapcs)
		LOGD("To 9151 len:%d, data:%s", datalen, buffer);
	else if(dev == uart_wifi)
		LOGD("To wifi len:%d, data:%s", datalen, buffer);
#endif

	uart_fifo_fill(dev, buffer, datalen);
	uart_irq_tx_enable(dev); 	
}

static void uart_receive_data_handle(struct device *dev, uint8_t *data, uint32_t datalen)
{
	if(dev == uart_wifi)
	{
		wifi_receive_data_handle(data, datalen);
	}
	else if(dev == uart_mapcs)
	{
	#ifdef CONFIG_PPG_SUPPORT
		if(strncmp(data, PPG_DATA_HEAD, strlen(PPG_DATA_HEAD)) == 0)
		{
			UartPPGEventHandle(data, datalen);
		}
	#endif

	#ifdef CONFIG_ECG_SUPPORT
		if(strncmp(data, ECG_DATA_HEAD, strlen(ECG_DATA_HEAD)) == 0)
		{
			UartECGEventHandle(data, datalen);
		}
	#endif

	#ifdef CONFIG_TEMP_SUPPORT
		if(strncmp(data, TEMP_DATA_HEAD, strlen(TEMP_DATA_HEAD)) == 0)
		{
			UartTempEventHandle(data, datalen);
		}
	#endif

	#ifdef CONFIG_WIFI_SUPPORT	
		if(strncmp(data, WIFI_DATA_HEAD, strlen(WIFI_DATA_HEAD)) == 0)
		{
			UartWifiEventHandle(data, datalen);
		}
	#endif

	#ifdef CONFIG_AUDIO_SUPPORT
		if(strncmp(data, AUDIO_DATA_HEAD, strlen(AUDIO_DATA_HEAD)) == 0)
		{
			UartAudioEventHandle(data, datalen);
		}
	#endif

	#ifdef CONFIG_BLE_SUPPORT
		if(strncmp(data, BLE_DATA_HEAD, strlen(BLE_DATA_HEAD)) == 0)
		{
			UartBleEventHandle(data, datalen);
		}
	#endif
	}
}

void UartMapcsSendData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_mapcs_send_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
	#ifdef UART_DEBUG
		LOGD("begin");
	#endif
		uart_send_data_handle(uart_mapcs, p_data, data_len);
		delete_data_from_cache(&uart_mapcs_send_cache);
		k_timer_start(&uart_mapcs_send_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void UartMapcsSendDataStart(void)
{
	k_timer_start(&uart_mapcs_send_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool MapcsSendCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_mapcs_send_cache))
		return true;
	else
		return false;
}

void MapcsSendData(UART_DATA_TYPE type, uint8_t *data, uint32_t datalen)
{
	int ret;
	uint8_t head_len, *ptr;

	ptr = k_malloc(datalen+UART_DATA_HEAD_MAX_LEN);
	if(ptr != NULL)
	{
		memset(ptr, 0x00, datalen+UART_DATA_HEAD_MAX_LEN);
		
		switch(type)
		{
		case UART_DATA_PPG:
			strcpy(ptr, PPG_DATA_HEAD);
			head_len = strlen(PPG_DATA_HEAD);
			break;
		case UART_DATA_ECG:
			strcpy(ptr, ECG_DATA_HEAD);
			head_len = strlen(ECG_DATA_HEAD);
			break;
		case UART_DATA_TEMP:
			strcpy(ptr, TEMP_DATA_HEAD);
			head_len = strlen(TEMP_DATA_HEAD);
			break;
		case UART_DATA_WIFI:
			strcpy(ptr, WIFI_DATA_HEAD);
			head_len = strlen(WIFI_DATA_HEAD);
			break;			
		case UART_DATA_AUIOD:
			strcpy(ptr, AUDIO_DATA_HEAD);
			head_len = strlen(AUDIO_DATA_HEAD);
			break;
		case UART_DATA_BLE:
			strcpy(ptr, BLE_DATA_HEAD);
			head_len = strlen(BLE_DATA_HEAD);
			break;
		}

		memcpy(ptr+head_len, data, datalen);
		ret = add_data_into_cache(&uart_mapcs_send_cache, ptr, datalen+head_len, DATA_TRANSFER);
		if(ret)
			UartMapcsSendDataStart();

		k_free(ptr);
	}
}

void UartMapcsReceData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_mapcs_rece_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
		uart_receive_data_handle(uart_mapcs, p_data, data_len);
		delete_data_from_cache(&uart_mapcs_rece_cache);
		k_timer_start(&uart_mapcs_rece_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void MapcsReceDataStart(void)
{
	k_timer_start(&uart_mapcs_rece_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool MapcsReceCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_mapcs_rece_cache))
		return true;
	else
		return false;
}

void UartMapcsReceFrameData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_mapcs_rece_cache, data, datalen, DATA_TRANSFER);
	MapcsReceDataStart();
}

static void uart_mapcs_cb(struct device *x)
{
	uint8_t tmpbyte = 0;
	uint32_t len=0;

	uart_irq_update(x);

	if(uart_irq_rx_ready(x)) 
	{
		if(uart_mapcs_rece_len >= BUF_MAXSIZE)
			uart_mapcs_rece_len = 0;

		while((len = uart_fifo_read(x, &uart_mapcs_rx_buf[uart_mapcs_rece_len], BUF_MAXSIZE-uart_mapcs_rece_len)) > 0)
		{
			uart_mapcs_rece_len += len;
			k_timer_start(&uart_mapcs_rece_frame_timer, K_MSEC(10), K_NO_WAIT);
		}
	}
	
	if(uart_irq_tx_ready(x))
	{
		struct uart_data_t *buf;
		uint16_t written = 0;

		buf = k_fifo_get(&fifo_uart_mapcs_tx_data, K_NO_WAIT);
		/* Nothing in the FIFO, nothing to send */
		if(!buf)
		{
			uart_irq_tx_disable(x);
			return;
		}

		while(buf->len > written)
		{
			written += uart_fifo_fill(x, &buf->data[written], buf->len - written);
		}

		while (!uart_irq_tx_complete(x))
		{
			/* Wait for the last byte to get
			* shifted out of the module
			*/
		}

		if (k_fifo_is_empty(&fifo_uart_mapcs_tx_data))
		{
			uart_irq_tx_disable(x);
		}

		k_free(buf);
	}
}

#ifdef CONFIG_WIFI_SUPPORT
void UartWifiSendData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_wifi_send_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
	#ifdef UART_DEBUG
		LOGD("begin");
	#endif
		uart_send_data_handle(uart_wifi, p_data, data_len);
		delete_data_from_cache(&uart_wifi_send_cache);
		k_timer_start(&uart_wifi_send_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void UartWifiSendDataStart(void)
{
	k_timer_start(&uart_wifi_send_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool WifiSendCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_wifi_send_cache))
		return true;
	else
		return false;
}

void WifiSendData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_wifi_send_cache, data, datalen, DATA_TRANSFER);
	if(ret)
		UartWifiSendDataStart();
}

void UartWifiReceData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_wifi_rece_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
		uart_receive_data_handle(uart_wifi, p_data, data_len);
		delete_data_from_cache(&uart_wifi_rece_cache);
		k_timer_start(&uart_wifi_rece_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void WifiReceDataStart(void)
{
	k_timer_start(&uart_wifi_rece_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool WifiReceCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_wifi_rece_cache))
		return true;
	else
		return false;
}

void UartWifiReceFrameData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_wifi_rece_cache, data, datalen, DATA_TRANSFER);
	WifiReceDataStart();
}

static void uart_wifi_cb(struct device *x)
{
	uint8_t tmpbyte = 0;
	uint32_t len=0;

	uart_irq_update(x);

	if(uart_irq_rx_ready(x)) 
	{
		if(uart_wifi_rece_len >= BUF_MAXSIZE)
			uart_wifi_rece_len = 0;

		while((len = uart_fifo_read(x, &uart_wifi_rx_buf[uart_wifi_rece_len], BUF_MAXSIZE-uart_wifi_rece_len)) > 0)
		{
			uart_wifi_rece_len += len;
			k_timer_start(&uart_wifi_rece_frame_timer, K_MSEC(10), K_NO_WAIT);
		}
	}
	
	if(uart_irq_tx_ready(x))
	{
		struct uart_data_t *buf;
		uint16_t written = 0;

		buf = k_fifo_get(&fifo_uart_wifi_tx_data, K_NO_WAIT);
		/* Nothing in the FIFO, nothing to send */
		if(!buf)
		{
			uart_irq_tx_disable(x);
			return;
		}

		while(buf->len > written)
		{
			written += uart_fifo_fill(x, &buf->data[written], buf->len - written);
		}

		while (!uart_irq_tx_complete(x))
		{
			/* Wait for the last byte to get
			* shifted out of the module
			*/
		}

		if (k_fifo_is_empty(&fifo_uart_wifi_tx_data))
		{
			uart_irq_tx_disable(x);
		}

		k_free(buf);
	}
}
#endif/*CONFIG_WIFI_SUPPORT*/

#ifdef CONFIG_PM_DEVICE
void uart_sleep_out(struct device *dev)
{
	if(dev == uart_mapcs)
	{
		if(k_timer_remaining_get(&uart_mapcs_sleep_in_timer) > 0)
			k_timer_stop(&uart_mapcs_sleep_in_timer);
		k_timer_start(&uart_mapcs_sleep_in_timer, K_SECONDS(UART_MAPCS_WAKE_HOLD_TIME_SEC), K_NO_WAIT);

		if(uart_mapcs_is_waked)
			return;

		uart_mapcs_is_waked = true;
	}
	else if(dev == uart_wifi)
	{
		if(uart_wifi_is_waked)
			return;

		uart_wifi_is_waked = true;
	}
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
	
#ifdef UART_DEBUG
	if(dev == uart_mapcs)
		LOGD("uart for 9151 set active success!");
	else if(dev == uart_wifi)
		LOGD("uart for wifi set active success!");
#endif
}

void uart_sleep_in(struct device *dev)
{	
	if(dev == uart_mapcs)
	{
		if(!uart_mapcs_is_waked)
			return;

		uart_mapcs_is_waked = false;
	}
	else if(dev == uart_wifi)
	{
		if(!uart_wifi_is_waked)
			return;

		uart_wifi_is_waked = false;
	}
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);

#ifdef UART_DEBUG
	if(dev == uart_mapcs)
		LOGD("uart for 9151 set low power success!");
	else if(dev == uart_wifi)
		LOGD("uart for wifi set low power success!");
#endif
}

static void mapcs_interrupt_event(struct device *interrupt, struct gpio_callback *cb, uint32_t pins)
{
	uart_mapcs_wake_flag = true;
}

static void UartMapcsSleepInCallBack(struct k_timer *timer_id)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif
	uart_mapcs_sleep_flag = true;
}

#ifdef CONFIG_WIFI_SUPPORT
static void UartWifiSleepInCallBack(struct k_timer *timer_id)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif
	uart_wifi_sleep_flag = true;
}
#endif/*CONFIG_WIFI_SUPPORT*/
#endif/*CONFIG_PM_DEVICE*/

static void UartMapcsSendDataCallBack(struct k_timer *timer)
{
	uart_mapcs_send_data_flag = true;
}

static void UartMapcsReceDataCallBack(struct k_timer *timer_id)
{
	uart_mapcs_rece_data_flag = true;
}

static void UartMapcsReceFrameCallBack(struct k_timer *timer_id)
{
	//uart_mapcs_rece_frame_flag = true;
	UartMapcsReceFrameData(uart_mapcs_rx_buf, uart_mapcs_rece_len);
	uart_mapcs_rece_len = 0;
}

#ifdef CONFIG_WIFI_SUPPORT
static void UartWifiSendDataCallBack(struct k_timer *timer)
{
	uart_wifi_send_data_flag = true;
}

static void UartWifiReceDataCallBack(struct k_timer *timer_id)
{
	uart_wifi_rece_data_flag = true;
}

static void UartWifiReceFrameCallBack(struct k_timer *timer_id)
{
	//uart_wifi_rece_frame_flag = true;
	UartWifiReceFrameData(uart_wifi_rx_buf, uart_wifi_rece_len);
	uart_wifi_rece_len = 0;
}
#endif

void UartWifiOff(void)
{
	if(k_timer_remaining_get(&uart_wifi_send_data_timer) > 0)
		k_timer_stop(&uart_wifi_send_data_timer);
	delete_all_from_cache(&uart_wifi_send_cache);
	
#ifdef CONFIG_PM_DEVICE
	uart_sleep_in(uart_wifi);
#endif
}

void uart_init(void)
{
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_UP;

#ifdef UART_DEBUG
	LOGD("begin");
#endif

	uart_mapcs = DEVICE_DT_GET(MAPCS_DEV);
	if(!uart_mapcs)
	{
	#ifdef UART_DEBUG
		LOGD("Could not get uart!");
	#endif
		return;
	}

	uart_irq_callback_set(uart_mapcs, uart_mapcs_cb);
	uart_irq_rx_enable(uart_mapcs);

	gpio_mapcs = DEVICE_DT_GET(MAPCS_PORT);
	if(!gpio_mapcs)
	{
	#ifdef UART_DEBUG
		LOGD("Could not get gpio!");
	#endif
		return;
	}	
	gpio_pin_configure(gpio_mapcs, MAPCS_WAKE_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_mapcs, MAPCS_WAKE_PIN, 1);

#ifdef CONFIG_PM_DEVICE
	gpio_pin_configure(gpio_mapcs, MAPCS_INT_PIN, flag);
	gpio_pin_interrupt_configure(gpio_mapcs, MAPCS_INT_PIN, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, mapcs_interrupt_event, BIT(MAPCS_INT_PIN));
	gpio_add_callback(gpio_mapcs, &gpio_cb);
	gpio_pin_interrupt_configure(gpio_mapcs, MAPCS_INT_PIN, GPIO_INT_ENABLE|GPIO_INT_EDGE_FALLING);	
	k_timer_start(&uart_mapcs_sleep_in_timer, K_SECONDS(UART_MAPCS_WAKE_HOLD_TIME_SEC), K_NO_WAIT);
#endif
}

#ifdef CONFIG_WIFI_SUPPORT
void uart_wifi_init(void)
{
	uart_wifi = DEVICE_DT_GET(WIFI_DEV);
	if(!uart_wifi)
	{
	#ifdef UART_DEBUG
		LOGD("Could not get uart!");
	#endif
		return;
	}
		
	uart_irq_callback_set(uart_wifi, uart_wifi_cb);
	uart_irq_rx_enable(uart_wifi);
}
#endif/*CONFIG_WIFI_SUPPORT*/

void UartMsgProc(void)
{
#ifdef CONFIG_PM_DEVICE
	if(uart_mapcs_wake_flag)
	{
		uart_mapcs_wake_flag = false;
		uart_sleep_out(uart_mapcs);
	}

	if(uart_mapcs_sleep_flag)
	{
		uart_mapcs_sleep_flag = false;
		uart_sleep_in(uart_mapcs);
	}

#ifdef CONFIG_WIFI_SUPPORT
	if(uart_wifi_wake_flag)
	{
		uart_wifi_wake_flag = false;
		uart_sleep_out(uart_wifi);
	}

	if(uart_wifi_sleep_flag)
	{
		uart_wifi_sleep_flag = false;
		uart_sleep_in(uart_wifi);
	}
#endif/*CONFIG_WIFI_SUPPORT*/
#endif/*CONFIG_PM_DEVICE*/

	if(uart_mapcs_send_data_flag)
	{
		UartMapcsSendData();
		uart_mapcs_send_data_flag = false;
	}

	if(uart_mapcs_rece_data_flag)
	{
		UartMapcsReceData();
		uart_mapcs_rece_data_flag = false;
	}
	
	if(uart_mapcs_rece_frame_flag)
	{
		UartMapcsReceFrameData(uart_mapcs_rx_buf, uart_mapcs_rece_len);
		memset(uart_mapcs_rx_buf, 0, sizeof(uart_mapcs_rx_buf));
		uart_mapcs_rece_len = 0;
		uart_mapcs_rece_frame_flag = false;
	}

#ifdef CONFIG_WIFI_SUPPORT
	if(uart_wifi_send_data_flag)
	{
		UartWifiSendData();
		uart_wifi_send_data_flag = false;
	}

	if(uart_wifi_rece_data_flag)
	{
		UartWifiReceData();
		uart_wifi_rece_data_flag = false;
	}

	if(uart_wifi_rece_frame_flag)
	{
		UartWifiReceFrameData(uart_wifi_rx_buf, uart_wifi_rece_len);
		memset(uart_wifi_rx_buf, 0, sizeof(uart_wifi_rx_buf));
		uart_wifi_rece_len = 0;
		uart_wifi_rece_frame_flag = false;
	}
#endif
}

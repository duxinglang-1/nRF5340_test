/****************************************Copyright (c)************************************************
** File Name:			    max86176.c
** Descriptions:			PPG AFE process source file
** Created By:				xie biao
** Created Date:			2025-07-07
** Modified Date:      		2025-07-07
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>
#include "ecg.h"
#include "max86176.h"
#include "uart.h"
#include "logger.h"

//#ifdef ECG_MAX86176

#define MAX86176_DEBUG

short CoeffBuf_40Hz_LowPass[FILTERORDER] = 
{
    1,    -6,   5,     2,    -12,   11,   8,     -26,  18,   20,    -49,  25,
    43,   -82,  28,    83,   -126,  22,   146,   -182, -2,   240,   -247, -56,
    377,  -317, -159,  573,  -387,  -343, 862,   -450, -679, 1341,  -501, -1407,
    2406, -535, -4188, 9276, 21299, 9276, -4188, -535, 2406, -1407, -501, 1341,
    -679, -450, 862,   -343, -387,  573,  -159,  -317, 377,  -56,   -247, 240,
    -2,   -182, 146,   22,   -126,  83,   28,    -82,  43,   25,    -49,  20,
    18,   -26,  8,     11,   -12,   2,    5,     -6,   1
};

short ECG_WorkingBuff[2 * FILTERORDER];

int EcgSampleCount = 0;

// 定义日志模块
LOG_MODULE_REGISTER(MAX86176, CONFIG_LOG_DEFAULT_LEVEL);

struct device *spi_ecg;
struct device *gpio_0_ecg,*gpio_1_ecg;
static struct gpio_callback gpio_cb;

static struct spi_buf_set tx_bufs, rx_bufs;
static struct spi_buf tx_buff, rx_buff;

static struct spi_config spi_cfg;
static struct spi_cs_control spi_cs_ctr;
bool ecg_int_flag = false;
bool max86176_power_ready = false;

bool gUseSpi = true;
uint8_t gReadBuf[AFE_FIFO_SIZE * NUM_BYTES_PER_SAMPLE] = {0}; // array to store register reads - size to hold full FIFO
bool gUseEcg = true;

// ============ 自适应增益控制变量 ============
static ecg_adaptive_gain_t adaptive_gain = 
{
    .enabled = true,                 // 默认启用自适应增益
    .current_gain = ECG_GAIN_LOW,    // 初始增益级别
    .signal_threshold_low = 10000,    // 信号幅度低于此值时增加增益
    .signal_threshold_high = 50000, // 信号幅度高于此值时降低增益
    .adjust_interval = 128,           // 修改成128降低波形的滞后性
    .sample_count = 0,
    .peak_value = 0,
    .abs_sum = 0,
    .high_count = 0,
    .holdoff_count = 0
};

// ECG 输入极性配置 (0: 正常, 1: 反向) - 对应寄存器 0x91 Bit 7
static uint8_t g_ecg_ipol = 1;

// Savitzky-Golay滤波器结构体和系数
typedef struct
{
  short buffer[9];    // 9点缓冲区
  int index;          // 当前写入位置
  int is_buffer_full; // 缓冲区是否已填满
} sg_filter_t;

static const double sg_coeffs[9] = {-0.0909, 0.0606, 0.1688, 0.2338, 0.2554, 0.2338, 0.1688, 0.0606, -0.0909};

static sg_filter_t sg_filter = {0};

static unsigned short ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1,
                      ECGFirstFlag = 1;

static int32_t ECG_Pvev_DC_Sample = 0;
static short ECG_Pvev_Sample = 0;
static int32_t ECG_DisplayHp_Sample = 0;
static short ECG_DisplayHp_PrevSample = 0;
static uint8_t ECG_DisplayHp_FirstFlag = 1;

// 滤波器状态重置函数
static void ECG_FilterState_Reset(void)
{
  int i;

  // 重置 IIR/FIR 滤波器状态
  ECGFirstFlag = 1; // 设置标志，下次调用 ECG_IIR_FIR_Filter 时会重新初始化
  ECG_bufStart = 0;
  ECG_bufCur = FILTERORDER - 1;
  ECG_Pvev_DC_Sample = 0;
  ECG_Pvev_Sample = 0;
  ECG_DisplayHp_Sample = 0;
  ECG_DisplayHp_PrevSample = 0;
  ECG_DisplayHp_FirstFlag = 1;

  // 清空工作缓冲区
  for (i = 0; i < (2 * FILTERORDER); i++)
  {
    ECG_WorkingBuff[i] = 0;
  }

  // 重置 Savitzky-Golay 滤波器
  memset(&sg_filter, 0, sizeof(sg_filter));

  // 重置采样计数
  EcgSampleCount = 0;
  
  // 重置 HR/HRV 计算
  ECG_HR_HRV_Reset();
}

static short ECG_Display_HighPassFilter(short sample)
{
  if (ECG_DisplayHp_FirstFlag)
  {
    ECG_DisplayHp_FirstFlag = 0;
    ECG_DisplayHp_Sample = 0;
    ECG_DisplayHp_PrevSample = sample;
    return 0;
  }

  int32_t t = ECG_DisplayHp_Sample + ((int32_t)sample - (int32_t)ECG_DisplayHp_PrevSample);
  ECG_DisplayHp_Sample = (t * ECG_DISPLAY_HP_ALPHA_Q15) >> 15;
  ECG_DisplayHp_PrevSample = sample;

  if (ECG_DisplayHp_Sample > INT16_MAX)
  {
    return INT16_MAX;
  }
  if (ECG_DisplayHp_Sample < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (short)ECG_DisplayHp_Sample;
}

static void ECG_Sleep_ms(int ms) { k_sleep(K_MSEC(ms)); }

static void Max86176_ResetFifoThreshold(void)
{
	Max86176_WriteReg(FIFO_CONFIGURATION_1_REGISTER,
	                  (uint8_t)(AFE_FIFO_SIZE - NUM_SAMPLES_PER_INT));
}

static void Max86176_FlushFifo(void)
{
	uint8_t fifo_cfg2 = 0;

	Max86176_ReadReg(FIFO_CONFIGURATION_2_REGISTER, 1, &fifo_cfg2);
	Max86176_WriteReg(FIFO_CONFIGURATION_2_REGISTER,
	                  fifo_cfg2 | FLUSH_FIFO_MASK | FIFO_STAT_CLR_MASK);
	ECG_Sleep_ms(2);
	Max86176_WriteReg(FIFO_CONFIGURATION_2_REGISTER,
	                  fifo_cfg2 & (uint8_t) ~(FLUSH_FIFO_MASK | FIFO_STAT_CLR_MASK));
}

static void Max86176_SendHrHrvReset(void)
{
	uint8_t hr_buffer[32] = {0};
	uint8_t hrv_buffer[32] = {0};

	strcpy(hr_buffer, COM_ECG_HR_DATA);
	strcat(hr_buffer, "0");
	MapcsSendData(UART_DATA_ECG, hr_buffer, strlen(hr_buffer));

	strcpy(hrv_buffer, COM_ECG_HRV_DATA);
	strcat(hrv_buffer, "0");
	MapcsSendData(UART_DATA_ECG, hrv_buffer, strlen(hrv_buffer));
}

static void ECG_QRS_RequestHoldoff(uint32_t samples);

static void ECG_CS_LOW(void) { gpio_pin_set(gpio_0_ecg, ECG_CS_PIN, 0); }

static void ECG_CS_HIGH(void) { gpio_pin_set(gpio_0_ecg, ECG_CS_PIN, 1); }

void ECG_Int_Event(void)
{ 
	if(max86176_power_ready)
	{
		ecg_int_flag = true;
	}
}

static void ECG_gpio_Init(void)
{
	int err;
	gpio_flags_t flag = GPIO_INPUT;

	// 端口初始化
	gpio_0_ecg = DEVICE_DT_GET(ECG_PORT0);
	if(!gpio_0_ecg)
	{
		return;
	}

	gpio_1_ecg = DEVICE_DT_GET(ECG_PORT1);
	if(!gpio_1_ecg)
	{
		return;
	}
	
	// 设置为高，模式为测量ecg,上电初始化的时候max86176供电必须先为低电平
	gpio_pin_configure(gpio_1_ecg, ECG_EN0_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_EN0_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_EN1_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_EN1_PIN, 0);

	gpio_pin_configure(gpio_1_ecg, ECG_I2C_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_I2C_CON_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_SPI_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_SPI_CON_PIN, 1);

	gpio_pin_configure(gpio_0_ecg, ECG_CS_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_CS_PIN, 1);

	// interrupt
	gpio_pin_configure(gpio_1_ecg, ECG_INT_PIN, flag);
	gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, ECG_Int_Event, BIT(ECG_INT_PIN));
	gpio_add_callback(gpio_1_ecg, &gpio_cb);
	gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN, GPIO_INT_ENABLE | GPIO_INT_EDGE_FALLING);
}

static void ECG_SPI_Init(void)
{
	spi_ecg = DEVICE_DT_GET(ECG_DEV);
	if(!spi_ecg)
	{
		return;
	}

	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	spi_cfg.frequency = 4000000;
	spi_cfg.slave = 0;
}

static void ECG_SPI_Transceive(uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen)
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

	ECG_CS_LOW();
	err = spi_transceive(spi_ecg, &spi_cfg, &tx_bufs, &rx_bufs);
	ECG_CS_HIGH();

	if(err)
	{
	}
}

void Max86176_WriteReg(uint8_t regAddr, uint8_t val)
{
	uint8_t i = 0;
	uint8_t tx_buf[8];

	tx_buf[i++] = regAddr;
	tx_buf[i++] = 0x00;
	tx_buf[i++] = val;
	ECG_SPI_Transceive(tx_buf, i, NULL, 0);
}

void Max86176_ReadReg(uint8_t regAddr, uint8_t numBytes, uint8_t *readbuf)
{
	uint8_t i = 0;
	uint8_t tx_buf[8];
	uint8_t rx_buf[numBytes + 2];

	tx_buf[i++] = regAddr;
	tx_buf[i++] = 0x80;
	ECG_SPI_Transceive(tx_buf, i, rx_buf, numBytes + 2);
	memcpy(readbuf, rx_buf + 2, numBytes);
}

void Max86176_StartEcg(void)
{
	gUseEcg = true;
	Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 4); // 修改采样率为 128sps
	k_sleep(K_MSEC(100));                                  // ECG启动稳定延时 (200ms→50ms)
}

void Max86176_StopEcg(void)
{
	gUseEcg = false;
	Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 4); // 修改采样率为128sps
}

/**
 * @brief 统一更新 ECG 配置寄存器 2 (0x91)
 * 包含：极性控制 (IPOL)、PGA 增益、INA 增益
 */
static void Max86176_RefreshEcgConfig2(void)
{
	// Bit 7: ECG_IPOL
	// Bit 6-4: ECG_PGA_GAIN (固定为 8x, 即 011b)
	// Bit 1-0: ECG_INA_GAIN (由 current_gain 决定)
	//uint8_t ecg_config2 = (g_ecg_ipol << 7) | (0x03 << 4) | (adaptive_gain.current_gain & 0x03);
	uint8_t ecg_config2 = (g_ecg_ipol << 7) | (0x03 << 4) | 0x00;
	Max86176_WriteReg(0x91, ecg_config2);

#ifdef MAX86176_DEBUG
	LOGD("Refresh Reg 0x91: 0x%02X (IPOL:%d, GainLevel:%d)", ecg_config2, g_ecg_ipol, adaptive_gain.current_gain);
#endif
}

void Max86176_Set_WearStatus(char *status)
{
#ifdef MAX86176_DEBUG
  	LOGD("Set_WearStatus:%s", status);
#endif

	// 根据佩戴位置自动调整极性 (重构逻辑)
  	if(strstr(status, "LEFT") != NULL)
  	{
		g_ecg_ipol = 1; // 左手佩戴：反向极性 (根据实际测试需求调整)
  	}
  	else if(strstr(status, "RIGHT") != NULL)
  	{
    	g_ecg_ipol = 0; // 右手佩戴：正常极性
	}
}

static void Sensor_Init(void) // call this on power up
{
	uint8_t part_id;
	uint8_t buffer[128] = {0};
	uint32_t len;

	Max86176_ReadReg(0xff, 1, &part_id);
#ifdef MAX86176_DEBUG
	LOGD("ID:%d", part_id);
#endif
	if(part_id != MAX86176_PART_ID)
		return;

	strcpy(buffer, COM_ECG_GET_INFOR);
	len = strlen(COM_ECG_GET_INFOR);
	sprintf(&buffer[len], "Sensor ID:%02X", part_id);
	MapcsSendData(UART_DATA_ECG, buffer, strlen(buffer));

	Max86176_WriteReg(0x10, 1); // RESET  ,并且使用过了内部时钟
	while(1)
	{
		Max86176_ReadReg(0x10, 1, &gReadBuf[0]);
	#ifdef MAX86176_DEBUG
		LOGD("reset:%d", gReadBuf[0]);
	#endif
		if((gReadBuf[0] & 0x01) == 0x00) // This bit then automatically becomes ‘0’ after thereset sequence is completed.
			break;
	}
  	k_sleep(K_MSEC(100)); // 等待模拟电路稳定

	uint8_t readbuf[6];
	Max86176_ReadReg(0x00, 6, readbuf); // 从0x00开始读6字节
#ifdef MAX86176_DEBUG
	LOGD("Reg0 0x%02X, 0x%02X,0x%02X, 0x%02X,0x%02X, 0x%02X", readbuf[0], readbuf[1], readbuf[2], readbuf[3], readbuf[4], readbuf[5]);
#endif

	Max86176_WriteReg(0x10, 0);
	Max86176_WriteReg(0x0d, AFE_FIFO_SIZE -	NUM_SAMPLES_PER_INT); // 设置FIFO数量，FIFO_A_FULL; assert
	// A_FULL on NUM_SAMPLES_PER_INT samples
	Max86176_WriteReg(0x1C, 0X20); // CLK_SEL; use internal 32.768kHz clock

	// PLL 配置: MDIV=0x1F, NDIV=0x13F (319)
	// PLL_CLK = FCLK * (MDIV + 288) / (NDIV + 1)
	// 32.768kHz * (31 + 288) / (319 + 1) = 32.768kHz * 319 / 320 ≈ 32.665kHz
	Max86176_WriteReg(0x19, 0x9F); // MDIV=0x1F, NDIV_MSB=1
	Max86176_WriteReg(0x1A, 0x3F); // NDIV_LSB=0x3F
	Max86176_WriteReg(0x18, 0x81); // PLL_EN=1

	// 等待 PLL 锁定，带超时保护（最大50次×100ms=5秒）
	uint8_t pll_status = 0;
	int pll_retry_count = 0;
	const int max_pll_retries = 50;

	Max86176_ReadReg(0x04, 1, &pll_status);
	while(!(pll_status & 0x02) && (pll_retry_count < max_pll_retries))
	{
		LOGD("pll_status : 0x%02X, retry: %d", pll_status, pll_retry_count);
		ECG_Sleep_ms(100);
		Max86176_ReadReg(0x04, 1, &pll_status);
		pll_retry_count++;
	}

	k_sleep(K_MSEC(100));

	adaptive_gain.current_gain = ECG_GAIN_LOW; // 初始增益级别 20x
	Max86176_RefreshEcgConfig2();             // 使用统一接口初始化增益和极性

	Max86176_WriteReg(0xa2, (0 << 7) | (0 << 6)); // ECG_P正极输入端和ECG_N负极输入端连接到ECG通道
						   // OPEN_P | OPEN_N

	Max86176_WriteReg(0x92, (2 << 6) | 0x3F);
	Max86176_WriteReg(0x93, (0b10 << 5) | (0 << 4) | 0x00);
	//可以修改最后一位0x05，其他配置正确
	Max86176_WriteReg(0x94, (0 << 7) | (1 << 6) | (2 << 4) | 0x05); // HI_CM_RES=1, Mode=AC+DC, IMAG=5
	Max86176_WriteReg(0x95, (1 << 7) | 0x02);                       // Sine wave, Fs/4 (256Hz) - 提高频率利于滤除杂波
	Max86176_WriteReg(0x96, (0b00 << 6) | 0x00);
	Max86176_WriteReg(0x98, 0x80);
	Max86176_WriteReg(0x99, 0x0D);
	Max86176_WriteReg(0x9A, (0b00 << 6) | 0x05);
	Max86176_WriteReg(0x85, 0x00);
}

/*
FIR滤波有限脉冲响应
*/
void ECG_FilterProcess(short *WorkingBuff, short *CoeffBuf, short *FilterOut)
{
	int i;
	int64_t acc = 0;
	for (i = 0; i < FILTERORDER; i++)
	{
		acc += (int32_t)(*CoeffBuf++) * (int32_t)(*WorkingBuff--);
	}

	int32_t out = (int32_t)(acc >> 15);
	if (out > INT16_MAX)
	{
		out = INT16_MAX;
	}
	else if (out < INT16_MIN)
	{
		out = INT16_MIN;
	}
	*FilterOut = (short)out; // 根据滤波器精度调整右移位数
}

void ECG_IIR_FIR_Filter(short CurrAqsSample, short *FilteredOut)
{
	short *CoeffBuf;
	int32_t temp1, temp2;
	short ECGData;
	unsigned short Cur_Chan;
	short FiltOut;

	CoeffBuf = CoeffBuf_40Hz_LowPass;
	if(ECGFirstFlag)
	{
		// 使用第一个采样值预填充滤波器状态，消除启动瞬态响应
		// 这样滤波器从稳定状态开始工作，避免抛物线波形
		for(Cur_Chan = 0; Cur_Chan < (2 * FILTERORDER); Cur_Chan++)
		{
			ECG_WorkingBuff[Cur_Chan] =	0; // FIR缓冲区初始化为0（因为去直流后信号围绕0）
		}
		// 预设IIR高通滤波器状态为当前采样值，使其立即稳定
		ECG_Pvev_DC_Sample = 0;          // 直流分量从0开始
		ECG_Pvev_Sample = CurrAqsSample; // 上一个采样值设为当前值
		ECGFirstFlag = 0;

		// 第一个采样点直接输出0（因为没有前一个差分值）
		FilteredOut[0] = 0;
		return;
	}

	temp1 = ECG_Pvev_DC_Sample + ((int32_t)CurrAqsSample - (int32_t)ECG_Pvev_Sample);
	ECG_Pvev_DC_Sample = (temp1 * ECG_QRS_HP_ALPHA_Q15) >> 15;
	ECG_Pvev_Sample = CurrAqsSample;
	temp2 = ECG_Pvev_DC_Sample >> 2;
	if(temp2 > INT16_MAX)
	{
		ECGData = INT16_MAX;
	}
	else if(temp2 < INT16_MIN)
	{
		ECGData = INT16_MIN;
	}
	else
	{
		ECGData = (short)temp2;
	}
	ECG_WorkingBuff[ECG_bufCur] = ECGData;
	ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (short *)&FiltOut);
	ECG_WorkingBuff[ECG_bufStart] = ECGData;
	FilteredOut[0] = FiltOut;
	ECG_bufCur++;
	ECG_bufStart++;
	if(ECG_bufStart == (FILTERORDER - 1))
	{
		ECG_bufStart = 0;
		ECG_bufCur = FILTERORDER - 1;
	}

	return;
}

short ECG_SavitzkyGolay_Filter(short new_sample)
{
	// 将新样本存入缓冲区
	sg_filter.buffer[sg_filter.index] = new_sample;
	sg_filter.index++;

	// 循环缓冲区
	if(sg_filter.index >= 9)
	{
		sg_filter.index = 0;
		sg_filter.is_buffer_full = 1;
	}

	// 只有当缓冲区填满后才进行滤波
	if(!sg_filter.is_buffer_full)
	{
		return new_sample; // 缓冲区未满时返回原始值
	}

	// 计算卷积（滤波）
	double sum = 0.0;
	int coeff_index = 0;

	// 从当前索引向前遍历缓冲区（循环）
	for(int i = sg_filter.index, count = 0; count < 9; count++)
	{
		sum += sg_filter.buffer[i] * sg_coeffs[coeff_index++];

		i++;
		if(i >= 9)
		{
		  i = 0; // 循环到缓冲区开头
		}
	}

	// 将结果转换回short类型，注意数据范围
	// int16_t result = (int16_t)(sum + 0.5); // 四舍五入
	int16_t result = (int16_t)(sum + (sum >= 0 ? 0.5f : -0.5f));

	return result;
}

short ECGFilteredData[4];

// ECG 数据批量发送缓冲区 (每 0.25 秒 32 个数据)
#define ECG_BATCH_SIZE 32
// 启动预热期：跳过前N个不稳定采样点
// 由于滤波器已采用状态预填充技术，预热期可以缩短至FIR阶数+SG滤波窗口
#define ECG_WARMUP_SAMPLES (16) // 大幅缩短预热期至 0.125s，快速出波形

static bool ecg_warmup_done = false;
static int ecg_warmup_counter = 0;

static short ecg_batch_buffer[ECG_BATCH_SIZE];
static uint32_t ecg_batch_count = 0;

// 调试计数器
static uint32_t dbg_int_count = 0;        // 中断触发次数
static uint32_t dbg_work_count = 0;       // 工作队列执行次数
static uint32_t dbg_fifo_ready_count = 0; // FIFO准备好次数
static uint32_t dbg_consumer_count = 0;   // 消费者读取次数

// AFE/HRV 相关状态
static int32_t gEcgSampleCount = -1;
static uint32_t g_hr_hrv_send_counter = 0; // HR/HRV 发送计数器

// ============ 导联状态跟踪 ============
static bool g_last_lead_status = false;

// 前向声明
static void Max86176_ParseLeadStatus(uint8_t status6, bool *lon, bool *ac_lead_connected);
static void Max86176_ReportLeadStatus(bool lead_connected, bool force_send);
bool Max86176_CheckAcLeadOffStatus(void);

// 使用k_work工作队列替代信号量+线程方案
static struct k_work afe_work;
static void afe_work_handler(struct k_work *work);

// 初始化工作队列
static bool afe_work_initialized = false;

static void afe_work_init(void)
{
	if(!afe_work_initialized)
	{
		k_work_init(&afe_work, afe_work_handler);
		afe_work_initialized = true;
	}
}

// status6(0x05) 同时包含两类导联状态：
// bit7 = LON，用于 ULP 模式判断“是否接触到电极”；
// bit5 = AC_LOFF，用于 ECG 模式判断“是否从已接触状态脱落”。
static void Max86176_ParseLeadStatus(uint8_t status6, bool *lon, bool *ac_lead_connected)
{
	if(lon != NULL)
	{
		*lon = ((status6 >> 7) & 0x01) ? true : false;
	}

	if(ac_lead_connected != NULL)
	{
		bool ac_loff = (status6 >> 5) & 0x01;
		*ac_lead_connected = !ac_loff;
	}
}

// ULP 导联检测模式只保留 LON 检测和对应中断，
// 在低功耗下等待用户重新贴附电极。
static void Max86176_EnterUlpLeadOnMode(void)
{
	// 清空批量发送缓冲区中的残留数据，防止下次启动时混入旧数据
	ecg_batch_count = 0;

	uint8_t status_regs[NUM_STATUS_REGS];

	k_work_cancel(&afe_work);
	//MapcsSendCacheClear();
	ecg_int_flag = false;
	gUseEcg = false;

	Max86176_StopEcg();
	k_sleep(K_MSEC(50));

	Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);
	Max86176_FlushFifo();
	Max86176_ResetFifoThreshold();
	
	ECG_Sleep_ms(10);
	// 0x93 仅开 LON 检测，0x85 仅开 LON 中断，关闭 FIFO A_FULL 中断。
	Max86176_WriteReg(0x93, (1 << 7));
	Max86176_WriteReg(0x85, (1 << 7));
	Max86176_WriteReg(0x80, 0x00); 

	gEcgSampleCount = 0;
	ecg_warmup_done = false;
	ecg_warmup_counter = 0;
	g_hr_hrv_send_counter = 0;
	ECG_FilterState_Reset();
	adaptive_gain.sample_count = 0;
	adaptive_gain.peak_value = 0;
}

// 进入 ECG 测量模式后，导联监控从 LON 切换到 AC lead-off 中断。
static void Max86176_EnterEcgMeasureMode(void)
{
	uint8_t status_regs[NUM_STATUS_REGS];

	k_work_cancel(&afe_work);
	//MapcsSendCacheClear();
	ecg_int_flag = false;
	gUseEcg = true;

	Max86176_StopEcg();
	k_sleep(K_MSEC(50));

	Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);
	Max86176_FlushFifo();
	Max86176_ResetFifoThreshold();
	
	ECG_Sleep_ms(10);

	Max86176_WriteReg(0xA8, (1<<7) | (1<<6) | (1<<4) | (1 << 3)| (1 << 2)| (1 << 1)| 1);
	Max86176_WriteReg(0xA9, (1 << 6) ); 
	Max86176_WriteReg(0x93, (0b10 << 5));
	Max86176_WriteReg(0x80, (1<<7));
	Max86176_WriteReg(0x85, 0x20);
	Max86176_ReadReg(0x98, 2, status_regs);
#ifdef MAX86176_DEBUG
	LOGD("REG 0x98:%02x, 0x99:%02x", status_regs[0], status_regs[1]);
#endif
	Max86176_StartEcg();
	k_sleep(K_MSEC(500));

	Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);
	Max86176_FlushFifo();
	Max86176_ResetFifoThreshold();
	
	ECG_Sleep_ms(10);

	ecg_batch_count = 0;
	gEcgSampleCount = 0;
	ecg_warmup_done = false;
	ecg_warmup_counter = 0;
	dbg_int_count = 0;
	dbg_work_count = 0;
	dbg_fifo_ready_count = 0;
	dbg_consumer_count = 0;
	g_hr_hrv_send_counter = 0;

	ECG_FilterState_Reset();
	Max86176_SendHrHrvReset();

	adaptive_gain.sample_count = 0;
	adaptive_gain.peak_value = 0;
}

// array to store ECG/ACLOFF/PPG ADC counts, time data
int32_t adcCountArr[NUM_ADC][NUM_SAMPLES_PER_INT * EXTRABUFFER]; // array to store ECG/ACLOFF/PPG ADC
                                           // counts, time data

// 静态变量用于AFE处理
#define HR_HRV_SEND_INTERVAL (ECG_SAMPLE_RATE * 2) // 每2秒发送一次 HR/HRV 数据

// AFE工作处理函数 - 由系统工作队列调度执行
// 采用循环处理方式，确保FIFO中所有数据都被处理，避免高频中断时数据丢失
static void afe_work_handler(struct k_work *work)
{
	int32_t ecg_value;
	uint8_t data0, data1, data2, sampleIx[NUM_ADC] = {0}, tag;
	uint16_t readBufIx = 0;
	uint32_t ecg_in_this_batch = 0;
	int loop_count = 0;
	const int max_loops = 10; // 防止无限循环，最多处理10批次

	dbg_work_count++;
	bool lead_connected = g_last_lead_status;

	// 导联脱落期间，清空已缓存的批量数据，防止残留噪声被发送
	if(!lead_connected)
	{
		ecg_batch_count = 0;
	}

	// 仅读取 INT_STATUS1(0x00) 判断 A_FULL，避免把 0x05(含导联状态)一并读清造成导联跳变
	uint8_t int_status1;
	Max86176_ReadReg(0x00, 1, &int_status1);

	if(!(int_status1 & 0x80))
		return; // 无FIFO数据需要处理

	// 循环处理直到FIFO为空或达到最大循环次数
	while(loop_count < max_loops)
	{
		//第一次循环使用已读取的状态，后续循环重新读取
		if(loop_count > 0)
		{
			Max86176_ReadReg(0x00, 1, &int_status1);
			if(!(int_status1 & 0x80))
				break; // FIFO为空，退出循环
		}

		// 重置每批次的sample索引
		memset(sampleIx, 0, sizeof(sampleIx));
		ecg_in_this_batch = 0;

		loop_count++;
		dbg_fifo_ready_count++;

		Max86176_ReadReg(0x0a, 2, gReadBuf); // read FIFO_DATA_COUNT
		uint8_t ovf_counter = (gReadBuf[0] >> 1) & 0x7F;
		if(ovf_counter > 0)
		{
			Max86176_FlushFifo();
			Max86176_ResetFifoThreshold();
			Max86176_ReadReg(0x00, 1, &int_status1);
			break; // 跳过本批数据
		}

		uint32_t count = ((gReadBuf[0] & 0x80) << 1) | gReadBuf[1]; // FIFO_DATA_COUNT (9-bit)

		// 安全检查：防止 count 超过 FIFO 大小
		if(count > AFE_FIFO_SIZE)
		{
			count = AFE_FIFO_SIZE;
		}

		Max86176_ReadReg(0x0c, count * NUM_BYTES_PER_SAMPLE, gReadBuf); // read FIFO_DATA
		for(readBufIx = 0; readBufIx < (count * NUM_BYTES_PER_SAMPLE);readBufIx += NUM_BYTES_PER_SAMPLE) // parse the FIFO data
		{
			tag = (gReadBuf[readBufIx] >> 4) & 0xf;
			if(tag == TAG_ECG)
			{
				// ============ 导联脱落期间：不滤波、不发送、不累积 ============
				if(!lead_connected)
				{
					ecg_batch_count = 0;
					continue;
				}

				data0 = gReadBuf[readBufIx];
				data1 = gReadBuf[readBufIx + 1];
				data2 = gReadBuf[readBufIx + 2];
				// ECG 数据是 18-bit 二补码
				ecg_value = ((data0 & 0x03) << 16) | (data1 << 8) | data2;

				// 18位二补码转换：如果bit17是1，则为负数
				int32_t ecg_signed = ecg_value;
				if(ecg_signed & 0x20000)
				{
					ecg_signed -= (1 << 18);
				}

				// 使用int32_t进行滤波，避免short溢出
				short curr_sample = 0;
				if(ecg_signed > INT16_MAX)
				{
					curr_sample = INT16_MAX;
				}
				else if(ecg_signed < INT16_MIN)
				{
					curr_sample = INT16_MIN;
				}
				else
				{
					curr_sample = (short)ecg_signed;
				}

				// Max86176_AdaptiveGainProcess(ecg_signed);

				// ============ 启动预热期处理 ============
				// 预热期将数据送入滤波器，使其快速收敛，但暂不发送
				static bool just_finished_warmup = false;
				if(!ecg_warmup_done)
				{
					// 依然执行滤波，让滤波器状态快速稳定
					ECG_IIR_FIR_Filter(curr_sample, &ECGFilteredData[1]);
					ECGFilteredData[1] = ECG_SavitzkyGolay_Filter(ECGFilteredData[1]);

					short ecg_qrs_sample = ECGFilteredData[1];
					short ecg_send_sample = ECG_Display_HighPassFilter(ecg_qrs_sample);

					// 预热期暂不发送数据，但存入缓冲区
					ecg_batch_buffer[ecg_batch_count] = ecg_send_sample;
					ecg_batch_count++;

					ecg_warmup_counter++;
					if(ecg_warmup_counter < ECG_WARMUP_SAMPLES)
					{
						continue;
					}
					else
					{
						// 预热完成
						ecg_warmup_done = true;
						just_finished_warmup = true;
					}
				}
				else
				{
					// 预热完成后正常滤波
					ECG_IIR_FIR_Filter(curr_sample, &ECGFilteredData[1]);
					ECGFilteredData[1] = ECG_SavitzkyGolay_Filter(ECGFilteredData[1]);

					short ecg_qrs_sample = ECGFilteredData[1];
					short ecg_send_sample = ECG_Display_HighPassFilter(ecg_qrs_sample);
					ecg_batch_buffer[ecg_batch_count] = ecg_send_sample;
					ecg_batch_count++;
				}

				// ============ QRS 检测和 HR/HRV 计算 ============
				ECG_Process_QRS_Detection(ECGFilteredData[1]);

				// ============ HR/HRV 数据发送 ============
				g_hr_hrv_send_counter++;
				if(g_hr_hrv_send_counter >= HR_HRV_SEND_INTERVAL)
				{
					g_hr_hrv_send_counter = 0;

					uint8_t hr = ECG_Get_Heart_Rate();
					uint16_t hrv = ECG_Get_HRV_SDNN();

					// 发送心率数据
					uint8_t hr_buffer[32] = {0};
					strcpy(hr_buffer, COM_ECG_HR_DATA);
					sprintf(hr_buffer + strlen(COM_ECG_HR_DATA), "%d", hr);
					MapcsSendData(UART_DATA_ECG, hr_buffer, strlen(hr_buffer));

					// 发送 HRV 数据
					uint8_t hrv_buffer[32] = {0};
					strcpy(hrv_buffer, COM_ECG_HRV_DATA);
					sprintf(hrv_buffer + strlen(COM_ECG_HRV_DATA), "%d", hrv);
					MapcsSendData(UART_DATA_ECG, hrv_buffer, strlen(hrv_buffer));
				}

				// 每收集 ECG_BATCH_SIZE 个数据后批量发送
				// 或者预热刚完成时立即发送，快速显示波形
				if(ecg_batch_count >= ECG_BATCH_SIZE || just_finished_warmup)
				{
					uint32_t len;
					uint8_t buffer[ECG_BATCH_SIZE * 2 + 10] = {0};

					strcpy(buffer, COM_ECG_GET_DATA);
					len = strlen(COM_ECG_GET_DATA);
					memcpy(&buffer[len], (void *)&ecg_batch_buffer, ecg_batch_count * 2);
					MapcsSendData(UART_DATA_ECG, buffer, ecg_batch_count * 2 + len);

					ecg_batch_count = 0;
					just_finished_warmup = false;
				}

				ecg_in_this_batch++;

				gEcgSampleCount++;
				adcCountArr[IX_ECG][sampleIx[IX_ECG]++] = (gReadBuf[readBufIx + 0] >> 2) & 1;
				adcCountArr[IX_ECG][sampleIx[IX_ECG]] = ((gReadBuf[readBufIx + 0] & 0x3) << 16) + (gReadBuf[readBufIx + 1] << 8) + gReadBuf[readBufIx + 2];
				if(gReadBuf[readBufIx + 0] & 0x2)
					adcCountArr[IX_ECG][sampleIx[IX_ECG]] -= (1 << 18);

				sampleIx[IX_ECG]++;
			}
			else if (tag == TAG_LOFFUTIL)
			{
				tag = (gReadBuf[readBufIx + 0] >> 2) & 1; // this can also be used for the array index in this example
				adcCountArr[tag][sampleIx[tag]] = ((gReadBuf[readBufIx + 1] & 0xf) << 8) + gReadBuf[readBufIx + 2];
				if(gReadBuf[readBufIx + 0] & 0x8)
					adcCountArr[tag][sampleIx[tag]] -= (1 << 12);

				// 复用已解析的 ADC 值
				int32_t raw = adcCountArr[tag][sampleIx[tag]];

				sampleIx[tag]++;
				if(sampleIx[tag] >= NUM_SAMPLES_PER_INT * EXTRABUFFER)
				{
					sampleIx[tag] = 0;
				}
			}
		}
	}

	// 如果达到最大循环次数仍有数据，重新调度自己
	if(loop_count >= max_loops)
		k_work_submit(&afe_work);
}

void Max86176_onAfeInt(void) // call this on AFE interrupt
{
	dbg_int_count++;

	// 提交工作到系统工作队列，异步执行耗时操作
	k_work_submit(&afe_work);
}

void Max86176_init(void)
{
	max86176_power_ready = false;
	ECG_gpio_Init();
	ECG_SPI_Init();
	// 初始化AFE工作队列
	afe_work_init();
	k_sleep(K_MSEC(50));
	Sensor_Init();
	// 启用自适应增益控制
	Max86176_EnableAdaptiveGain(false);
}

void Max86176_start_measure(void)
{
	g_last_lead_status = false;
	Max86176_EnterUlpLeadOnMode();
}

void Max86176_stop(void)
{
	max86176_power_ready = false;

	// 取消已提交但未执行的工作队列，防止停止后再处理FIFO数据
	k_work_cancel(&afe_work);

	// 停止 ECG 采集
	Max86176_StopEcg();

	// 清除中断标志
	ecg_int_flag = false;

	// 清除状态寄存器
	Max86176_ReadReg(0x00, NUM_STATUS_REGS, gReadBuf);
	Max86176_FlushFifo();
	Max86176_ResetFifoThreshold();

	// 清空批量发送缓冲区中的残留数据，防止下次启动时混入旧数据
	ecg_batch_count = 0;
	g_hr_hrv_send_counter = 0;
	ECG_FilterState_Reset();
	Max86176_SendHrHrvReset();

	// 清空UART发送缓存中的残留ECG数据
	//MapcsSendCacheClear();
	gpio_pin_configure(gpio_1_ecg, ECG_EN0_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_EN0_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_EN1_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_EN1_PIN, 0);

	gpio_pin_configure(gpio_1_ecg, ECG_I2C_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_I2C_CON_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_SPI_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_SPI_CON_PIN, 0);

	MapcsSendData(UART_DATA_ECG, COM_ECG_SET_CLOSE, strlen(COM_ECG_SET_CLOSE));
}

void Max86176_start(void)
{
	uint8_t part_id;

	ecg_int_flag = false;
	max86176_power_ready = false;

	// 确保 GPIO 配置正确
	gpio_pin_configure(gpio_1_ecg, ECG_EN0_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_EN0_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_EN1_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_EN1_PIN, 1);

	gpio_pin_configure(gpio_1_ecg, ECG_I2C_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_I2C_CON_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_SPI_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_SPI_CON_PIN, 1);

	// 待机唤醒后需要更长时间等待模拟电路稳定 (从 50ms 增加到 100ms)
	k_sleep(K_MSEC(150));

	// 检查传感器是否在线（读取 ID 寄存器）
	Max86176_ReadReg(0xff, 1, &part_id); 
#ifdef MAX86176_DEBUG
	LOGD("ID: 0x%02X", part_id);
#endif
	if(part_id != MAX86176_PART_ID)
		return;

	// 初始化 AFE 工作队列 (只初始化一次)
	afe_work_init();

	Sensor_Init();

	// 传感器复位后需要足够时间让模拟前端稳定 (从 50ms 增加到 150ms)
	k_sleep(K_MSEC(150));

	// 启用自适应增益控制
	Max86176_EnableAdaptiveGain(false);

	k_sleep(K_MSEC(30)); // 等待 RLD 和增益配置生效 (50ms→30ms)

	// 再次清除可能在初始化期间产生的虚假中断标志
	ecg_int_flag = false;

	// 最后启动测量 (内部会重新启用中断)
	max86176_power_ready = true;
	Max86176_start_measure();

	MapcsSendData(UART_DATA_ECG, COM_ECG_SET_OPEN, strlen(COM_ECG_SET_OPEN));
}

// ============ ECG增益控制实现 ============

/**
 * @brief 设置ECG增益级别
 * @param level 增益级别 (ECG_GAIN_LOW ~ ECG_GAIN_VERY_HIGH)
 * 
 * INA增益映射:
 *   0 -> 20x,  1 -> 40x,  2 -> 80x,  3 -> 160x
 * PGA增益固定为8x
 */
void Max86176_SetEcgGain(ecg_gain_level_t level)
{
	if(level > ECG_GAIN_VERY_HIGH)
		level = ECG_GAIN_VERY_HIGH;

	adaptive_gain.current_gain = level;
	Max86176_RefreshEcgConfig2();
	ECG_QRS_RequestHoldoff(64);

#ifdef MAX86176_DEBUG
	LOGD("ECG gain set to level %d (INA=%dx, total=%dx)", level, 20 << level, (20 << level) * 8);
#endif
}

/**
 * @brief 获取当前ECG增益级别
 */
ecg_gain_level_t Max86176_GetEcgGain(void)
{
  return adaptive_gain.current_gain;
}

/**
 * @brief 启用/禁用自适应增益控制
 */
void Max86176_EnableAdaptiveGain(bool enable)
{
	adaptive_gain.enabled = enable;
	adaptive_gain.sample_count = 0;
	adaptive_gain.peak_value = 0;
	adaptive_gain.abs_sum = 0;
	adaptive_gain.high_count = 0;
	adaptive_gain.holdoff_count = 0;

#ifdef MAX86176_DEBUG	
	LOGD("Adaptive gain control %s", enable ? "enabled" : "disabled");
#endif
}

/**
 * @brief 自适应增益处理
 * @param ecg_value 当前ECG采样值(有符号)
 * 
 * 在每个ECG采样点调用此函数，会自动追踪信号幅度并在必要时调整增益
 */
void Max86176_AdaptiveGainProcess(int32_t ecg_value)
{
	if(!adaptive_gain.enabled)
		return;

	// 取绝对值追踪峰值
	int32_t abs_value = ecg_value >= 0 ? ecg_value : -ecg_value;
	if(abs_value > adaptive_gain.peak_value)
	{
		adaptive_gain.peak_value = abs_value;
	}
	adaptive_gain.abs_sum += (uint32_t)abs_value;
	if (abs_value > adaptive_gain.signal_threshold_high)
	{
		adaptive_gain.high_count++;
	}

	adaptive_gain.sample_count++;

	// 达到评估间隔时进行增益调整判断
	if(adaptive_gain.sample_count >= adaptive_gain.adjust_interval)
	{
		ecg_gain_level_t new_gain = adaptive_gain.current_gain;

		if (adaptive_gain.holdoff_count == 0)
		{
			if(adaptive_gain.peak_value > adaptive_gain.signal_threshold_high &&
			   adaptive_gain.high_count >= 3)
			{
				if(adaptive_gain.current_gain > ECG_GAIN_LOW)
				{
					new_gain = adaptive_gain.current_gain - 1;
				#ifdef MAX86176_DEBUG	
					LOGD("ECG signal strong (peak=%d), decreasing gain", adaptive_gain.peak_value);
				#endif
				}
			}
			else if(adaptive_gain.peak_value < adaptive_gain.signal_threshold_low)
			{
				if(adaptive_gain.current_gain < ECG_GAIN_VERY_HIGH)
				{
					new_gain = adaptive_gain.current_gain + 1;
				#ifdef MAX86176_DEBUG	
					LOGD("ECG signal weak (peak=%d), increasing gain", adaptive_gain.peak_value);
				#endif
				}
			}
		}

		// 如果增益需要改变，则设置新增益
		if(new_gain != adaptive_gain.current_gain)
		{
			Max86176_SetEcgGain(new_gain);
			adaptive_gain.holdoff_count = (uint16_t)(adaptive_gain.adjust_interval * 2);
		}
		else if (adaptive_gain.holdoff_count > 0)
		{
			if (adaptive_gain.holdoff_count > adaptive_gain.adjust_interval)
			{
				adaptive_gain.holdoff_count -= adaptive_gain.adjust_interval;
			}
			else
			{
				adaptive_gain.holdoff_count = 0;
			}
		}

		// 重置计数器和峰值
	#ifdef MAX86176_DEBUG
		LOGD("AGC window peak=%d, count=%u, high_count=%u, gain=%d",
		     adaptive_gain.peak_value, adaptive_gain.sample_count,
		     adaptive_gain.high_count, adaptive_gain.current_gain);
	#endif
		adaptive_gain.sample_count = 0;
		adaptive_gain.peak_value = 0;
		adaptive_gain.abs_sum = 0;
		adaptive_gain.high_count = 0;
	}
}

static void Max86176_ReportLeadStatus(bool lead_connected, bool force_send)
{
  if (force_send || (lead_connected != g_last_lead_status))
  {
    LOGD("Lead connected: %d", lead_connected);
    uint8_t buffer[64] = {0};

    strcpy(buffer, COM_ECG_LEAD_STATUS);
    if (lead_connected)
    {
      if (!g_last_lead_status)
      {
        // 导联从断开恢复到接通时，重置滤波和缓存，避免混入旧波形残留。
        ECG_FilterState_Reset();
        ecg_warmup_done = false;
        ecg_warmup_counter = 0;
        ecg_batch_count = 0;
      }
      strcat(buffer, COM_ECG_LEAD_ON);
    }
    else
    {
      ecg_batch_count = 0;
      g_hr_hrv_send_counter = 0;
      Max86176_SendHrHrvReset();
      strcat(buffer, COM_ECG_LEAD_OFF);
    }

    if (lead_connected)
    {
      LOGD("Send ECG lead on");
    }
    MapcsSendData(UART_DATA_ECG, buffer, strlen(buffer));
    g_last_lead_status = lead_connected;
  }
}
/**
 * @brief 检查DC导联脱落状态（独立读取版本，供外部调用）
 * @return true: 导联连接良好(无脱落), false: 导联脱落
 */
bool Max86176_CheckAcLeadOffStatus(void)
{
  uint8_t status6;
  bool lead_connected = false;
  Max86176_ReadReg(0x05, 1, &status6);
  Max86176_ParseLeadStatus(status6, NULL, &lead_connected);
  return lead_connected;
}

void Max86176_Msg_Process(void)
{
	uint8_t status6 = 0;
	static uint8_t lon_count = 0;
	static uint8_t loff_count = 0;

	if(ecg_int_flag)
	{
		ecg_int_flag = false;

		// 这里只读取导联状态寄存器，避免提前读取 0x00 清掉 A_FULL，
		// FIFO 是否需要处理交给 afe_work_handler() 自行判断。
		Max86176_ReadReg(0x05, 1, &status6);

		switch(status6 & 0xA0)//lead on status
		{
		case 0x80://LON
			if(!gUseEcg)
			{
				lon_count = 0;
				Max86176_EnterEcgMeasureMode();
				Max86176_ReportLeadStatus(true, false);
			}
			break;
			
		case 0x20://AC_LOFF
			loff_count++;
			
			if(gUseEcg && (loff_count > 2))
			{
				loff_count = 0;
				Max86176_EnterUlpLeadOnMode();
				Max86176_ReportLeadStatus(false, false);
			}
			break;
		}

		if(gUseEcg)
		{
			Max86176_onAfeInt();
		}
	}
}

// ============ 心率和 HRV 计算实现 ============

// QRS 检测相关变量
static int32_t qrs_det_ecg_buf[ECG_SAMPLE_RATE];	// ECG 数据缓冲区（1秒数据）
static int32_t qrs_det_deriv_buf[ECG_SAMPLE_RATE];	// 导数缓冲区
static int32_t qrs_det_sqr_buf[ECG_SAMPLE_RATE];	// 平方后数据缓冲区
static int32_t qrs_det_mwi_buf[ECG_SAMPLE_RATE];	// 移动窗口积分缓冲区
static uint32_t qrs_det_buf_idx = 0;				// 缓冲区索引
static int32_t qrs_det_threshold_i1 = 0;			// 集成数据阈值I1
static int32_t qrs_det_spk_i1 = 0;					// 集成信号峰值SPKI
static int32_t qrs_det_noise_i1 = 0;				// 集成噪声峰值NPKI
static uint32_t qrs_det_rr_intervals[HRV_RR_BUFFER_SIZE];	// RR 间期缓冲区（采样点）
static uint8_t qrs_det_rr_idx = 0;					// RR 缓冲区索引
static uint8_t qrs_det_rr_count = 0;				// RR 间期计数
static uint32_t qrs_det_last_r_peak_sample = 0;	// 上次 R 波的采样点计数
static uint32_t qrs_det_last_rr_samples = 0;
static uint32_t qrs_det_sample_count = 0;			// 总采样点计数
static bool qrs_det_in_refractory = false;			// 是否处于不应期
static uint32_t qrs_det_refractory_samples = 0;	// 不应期采样点数
static uint8_t qrs_det_current_hr = 0;				// 当前心率（bpm）
static bool qrs_det_initialized = false;			// 是否已初始化阈值
static uint32_t qrs_det_candidate_peak_sample = 0;
static int32_t qrs_det_candidate_peak_value = 0;
static bool qrs_det_candidate_valid = false;
static uint32_t qrs_det_holdoff_samples = 0;

static void ECG_QRS_RequestHoldoff(uint32_t samples)
{
	if (samples > qrs_det_holdoff_samples)
	{
		qrs_det_holdoff_samples = samples;
	}
}

// 初始化或重置 HR/HRV 计算
void ECG_HR_HRV_Reset(void)
{
	memset(qrs_det_ecg_buf, 0, sizeof(qrs_det_ecg_buf));
	memset(qrs_det_deriv_buf, 0, sizeof(qrs_det_deriv_buf));
	memset(qrs_det_sqr_buf, 0, sizeof(qrs_det_sqr_buf));
	memset(qrs_det_mwi_buf, 0, sizeof(qrs_det_mwi_buf));
	memset(qrs_det_rr_intervals, 0, sizeof(qrs_det_rr_intervals));
	
	qrs_det_buf_idx = 0;
	qrs_det_threshold_i1 = 0;
	qrs_det_spk_i1 = 0;
	qrs_det_noise_i1 = 0;
	qrs_det_rr_idx = 0;
	qrs_det_rr_count = 0;
	qrs_det_last_r_peak_sample = 0;
	qrs_det_last_rr_samples = 0;
	qrs_det_sample_count = 0;
	qrs_det_in_refractory = false;
	qrs_det_refractory_samples = (ECG_SAMPLE_RATE * 60) / HR_MAX; // 最大心率对应的最小间隔
	qrs_det_current_hr = 0;
	qrs_det_initialized = false;
	qrs_det_candidate_peak_sample = 0;
	qrs_det_candidate_peak_value = 0;
	qrs_det_candidate_valid = false;
}

static uint32_t ECG_QRS_GetRrAvgSamples(void)
{
	if (qrs_det_rr_count == 0)
	{
		return qrs_det_last_rr_samples;
	}

	uint8_t n = qrs_det_rr_count;
	if (n > 8u)
	{
		n = 8u;
	}

	uint64_t sum = 0;
	uint8_t idx =
		(uint8_t)((qrs_det_rr_idx + HRV_RR_BUFFER_SIZE - 1u) % HRV_RR_BUFFER_SIZE);
	for (uint8_t i = 0; i < n; i++)
	{
		sum += qrs_det_rr_intervals[idx];
		idx = (uint8_t)((idx + HRV_RR_BUFFER_SIZE - 1u) % HRV_RR_BUFFER_SIZE);
	}
	return (uint32_t)(sum / n);
}

// Pan-Tompkins 算法处理函数
void ECG_Process_QRS_Detection(short ecg_sample)
{
	int32_t i, sum;
	int32_t ecg_val = (int32_t)ecg_sample;
	
	// 1. 存储原始 ECG 数据到环形缓冲区
	qrs_det_ecg_buf[qrs_det_buf_idx] = ecg_val;
	
	// 2. 计算导数（5点导数）
	if (qrs_det_sample_count >= 4)
	{
		uint32_t prev4 = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 4) % ECG_SAMPLE_RATE;
		uint32_t prev3 = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 3) % ECG_SAMPLE_RATE;
		uint32_t prev2 = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 2) % ECG_SAMPLE_RATE;
		uint32_t prev1 = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 1) % ECG_SAMPLE_RATE;
		
		// y(n) = (2x(n) + x(n-1) - x(n-3) - 2x(n-4)) / 8
		int32_t deriv = (2 * ecg_val + 
						qrs_det_ecg_buf[prev1] - 
						qrs_det_ecg_buf[prev3] - 
						2 * qrs_det_ecg_buf[prev4]) / 8;
		qrs_det_deriv_buf[qrs_det_buf_idx] = deriv;
		
		// 3. 平方
		qrs_det_sqr_buf[qrs_det_buf_idx] = deriv * deriv;
		
		// 4. 移动窗口积分（150ms窗口：128Hz * 0.15 ≈ 19个点）
		#define MWI_WINDOW_SIZE 19
		sum = 0;
		for (i = 0; i < MWI_WINDOW_SIZE; i++)
		{
			uint32_t idx = (qrs_det_buf_idx + ECG_SAMPLE_RATE - i) % ECG_SAMPLE_RATE;
			sum += qrs_det_sqr_buf[idx];
		}
		qrs_det_mwi_buf[qrs_det_buf_idx] = sum / MWI_WINDOW_SIZE;
	}
	
	if (qrs_det_holdoff_samples > 0)
	{
		qrs_det_holdoff_samples--;
		qrs_det_buf_idx = (qrs_det_buf_idx + 1) % ECG_SAMPLE_RATE;
		qrs_det_sample_count++;
		return;
	}

	// 5. 初始化阶段：收集前1秒的数据来建立初始阈值
	if (!qrs_det_initialized && qrs_det_sample_count >= ECG_SAMPLE_RATE)
	{
		// 计算初始信号和噪声水平
		int32_t max_val = 0;
		int32_t min_val = INT32_MAX;
		int64_t sum_val = 0;
		
		for (i = 0; i < ECG_SAMPLE_RATE; i++)
		{
			int32_t val = qrs_det_mwi_buf[i];
			sum_val += val;
			if (val > max_val) max_val = val;
			if (val < min_val) min_val = val;
		}
		
		// 初始化峰值和噪声
		qrs_det_spk_i1 = max_val;
		qrs_det_noise_i1 = (int32_t)(sum_val / ECG_SAMPLE_RATE);
		qrs_det_threshold_i1 = qrs_det_noise_i1 + (qrs_det_spk_i1 - qrs_det_noise_i1) / 4;
		qrs_det_initialized = true;
#ifdef MAX86176_DEBUG
		LOGD("QRS init: spk=%d noise=%d th=%d", qrs_det_spk_i1,
		     qrs_det_noise_i1, qrs_det_threshold_i1);
#endif
	}
	
	// 6. 阈值和峰值检测（只在初始化完成后开始）
	if (qrs_det_initialized && qrs_det_sample_count >= ECG_SAMPLE_RATE + 2)
	{
		// 我们检测前一个点是否为峰值（因为需要比较前前、前、当前三个点）
		uint32_t check_idx = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 1) % ECG_SAMPLE_RATE;
		uint32_t check_prev_idx = (qrs_det_buf_idx + ECG_SAMPLE_RATE - 2) % ECG_SAMPLE_RATE;
		int32_t mwi_val = qrs_det_mwi_buf[check_idx];
		int32_t mwi_prev = qrs_det_mwi_buf[check_prev_idx];
		int32_t mwi_curr = qrs_det_mwi_buf[qrs_det_buf_idx];
		int32_t threshold_i2 = qrs_det_threshold_i1 / 2;
		bool is_peak = (mwi_val > mwi_prev && mwi_val > mwi_curr);
		
		// 检测是否处于不应期
		if (qrs_det_in_refractory)
		{
			qrs_det_refractory_samples--;
			if (qrs_det_refractory_samples == 0)
			{
				qrs_det_in_refractory = false;
			}
		}
		
		// 检测峰值（前一个点大于前前点和当前点，且超过阈值）
		if (!qrs_det_in_refractory && is_peak && mwi_val > qrs_det_threshold_i1)
		{
			// 检测到 QRS 波
			qrs_det_spk_i1 = (qrs_det_spk_i1 * 7 + mwi_val) / 8; // 更新峰值
			qrs_det_candidate_peak_sample = 0;
			qrs_det_candidate_peak_value = 0;
			qrs_det_candidate_valid = false;
			
			// 计算 RR 间期（峰值发生在前一个采样点）
			uint32_t peak_sample_count = qrs_det_sample_count - 1;
			if (qrs_det_last_r_peak_sample > 0)
			{
				uint32_t rr_samples = peak_sample_count - qrs_det_last_r_peak_sample;
				
				// 验证 RR 间期合理性（对应心率范围 30-220 bpm）
				uint32_t min_rr = (ECG_SAMPLE_RATE * 60) / HR_MAX;
				uint32_t max_rr = (ECG_SAMPLE_RATE * 60) / HR_MIN;
				
				if (rr_samples >= min_rr && rr_samples <= max_rr)
				{
					// 存储 RR 间期
					qrs_det_rr_intervals[qrs_det_rr_idx] = rr_samples;
					qrs_det_rr_idx = (qrs_det_rr_idx + 1) % HRV_RR_BUFFER_SIZE;
					if (qrs_det_rr_count < HRV_RR_BUFFER_SIZE)
					{
						qrs_det_rr_count++;
					}
					qrs_det_last_rr_samples = rr_samples;
					
					// 计算心率
					qrs_det_current_hr = (uint8_t)((ECG_SAMPLE_RATE * 60) / rr_samples);
				}
			}
			
			qrs_det_last_r_peak_sample = peak_sample_count;
			qrs_det_in_refractory = true;
			qrs_det_refractory_samples = (ECG_SAMPLE_RATE * 60) / HR_MAX; // 重置不应期
		}
		else
		{
			if (!qrs_det_in_refractory && is_peak && mwi_val > threshold_i2)
			{
				uint32_t peak_sample_count = qrs_det_sample_count - 1;
				if (!qrs_det_candidate_valid || mwi_val > qrs_det_candidate_peak_value)
				{
					qrs_det_candidate_peak_sample = peak_sample_count;
					qrs_det_candidate_peak_value = mwi_val;
					qrs_det_candidate_valid = true;
				}
			}

			if (mwi_val <= qrs_det_threshold_i1)
			{
				qrs_det_noise_i1 = (qrs_det_noise_i1 * 7 + mwi_val) / 8;
			}
		}

		if (qrs_det_last_r_peak_sample > 0)
		{
			uint32_t min_rr = (ECG_SAMPLE_RATE * 60) / HR_MAX;
			uint32_t max_rr = (ECG_SAMPLE_RATE * 60) / HR_MIN;
			uint32_t rr_avg = ECG_QRS_GetRrAvgSamples();
			if (rr_avg == 0)
			{
				rr_avg = (ECG_SAMPLE_RATE * 60u) / 75u;
			}
			uint32_t search_back_limit = (rr_avg * 166u) / 100u;
			if (search_back_limit < min_rr)
			{
				search_back_limit = min_rr;
			}
			if (search_back_limit > max_rr)
			{
				search_back_limit = max_rr;
			}

			uint32_t latest_sample = qrs_det_sample_count - 1;
			if ((latest_sample - qrs_det_last_r_peak_sample) > search_back_limit)
			{
				bool accepted = false;
				if (qrs_det_candidate_valid &&
				    qrs_det_candidate_peak_sample > qrs_det_last_r_peak_sample)
				{
					uint32_t rr_samples =
						qrs_det_candidate_peak_sample - qrs_det_last_r_peak_sample;
					if (rr_samples >= min_rr && rr_samples <= max_rr &&
					    qrs_det_candidate_peak_value > threshold_i2)
					{
						qrs_det_spk_i1 =
							(qrs_det_spk_i1 * 7 + qrs_det_candidate_peak_value) / 8;

						qrs_det_rr_intervals[qrs_det_rr_idx] = rr_samples;
						qrs_det_rr_idx =
							(uint8_t)((qrs_det_rr_idx + 1u) % HRV_RR_BUFFER_SIZE);
						if (qrs_det_rr_count < HRV_RR_BUFFER_SIZE)
						{
							qrs_det_rr_count++;
						}
						qrs_det_last_rr_samples = rr_samples;
						qrs_det_current_hr =
							(uint8_t)((ECG_SAMPLE_RATE * 60u) / rr_samples);

#ifdef MAX86176_DEBUG
						LOGD("QRS search-back: rr=%u hr=%u mwi=%d th=%d",
						     rr_samples, qrs_det_current_hr,
						     qrs_det_candidate_peak_value, qrs_det_threshold_i1);
#endif

						qrs_det_last_r_peak_sample = qrs_det_candidate_peak_sample;

						uint32_t elapsed_since_candidate =
							latest_sample - qrs_det_candidate_peak_sample;
						if (elapsed_since_candidate >= min_rr)
						{
							qrs_det_in_refractory = false;
							qrs_det_refractory_samples = 0;
						}
						else
						{
							qrs_det_in_refractory = true;
							qrs_det_refractory_samples =
								(min_rr - elapsed_since_candidate);
						}

						accepted = true;
					}
				}

				if (!accepted)
				{
					qrs_det_spk_i1 = (qrs_det_spk_i1 + qrs_det_noise_i1) / 2;
#ifdef MAX86176_DEBUG
					LOGD("QRS search-back drop th: spk=%d noise=%d th=%d",
					     qrs_det_spk_i1, qrs_det_noise_i1, qrs_det_threshold_i1);
#endif
				}

				qrs_det_candidate_peak_sample = 0;
				qrs_det_candidate_peak_value = 0;
				qrs_det_candidate_valid = false;
			}

			if ((latest_sample - qrs_det_last_r_peak_sample) > (max_rr * 2u))
			{
				qrs_det_current_hr = 0;
				qrs_det_spk_i1 = qrs_det_noise_i1;
				qrs_det_candidate_peak_sample = 0;
				qrs_det_candidate_peak_value = 0;
				qrs_det_candidate_valid = false;
			}
		}
		
		// 更新阈值
		qrs_det_threshold_i1 = qrs_det_noise_i1 + (qrs_det_spk_i1 - qrs_det_noise_i1) / 4;
	}
	
	// 更新缓冲区索引
	qrs_det_buf_idx = (qrs_det_buf_idx + 1) % ECG_SAMPLE_RATE;
	qrs_det_sample_count++;
}

// 获取当前心率（bpm）
uint8_t ECG_Get_Heart_Rate(void)
{
	return qrs_det_current_hr;
}

static uint32_t ECG_Isqrt_U32(uint32_t x)
{
	uint32_t result = 0;
	uint32_t bit = 1u << 30;

	while (bit > x)
	{
		bit >>= 2;
	}

	while (bit != 0)
	{
		if (x >= result + bit)
		{
			x -= result + bit;
			result = (result >> 1) + bit;
		}
		else
		{
			result >>= 1;
		}
		bit >>= 2;
	}

	return result;
}

// 计算 HRV 的 SDNN（标准差，单位：毫秒）
uint16_t ECG_Get_HRV_SDNN(void)
{
	if (qrs_det_rr_count < 2)
	{
		return 0;
	}

	uint16_t rr_ms_buf[HRV_RR_BUFFER_SIZE];
	uint8_t rr_ms_count = 0;

	const uint16_t min_rr_ms = (uint16_t)(60000u / HR_MAX);
	const uint16_t max_rr_ms = (uint16_t)(60000u / HR_MIN);
	uint8_t rr_start_idx =
		(uint8_t)((qrs_det_rr_idx + HRV_RR_BUFFER_SIZE - qrs_det_rr_count) %
		          HRV_RR_BUFFER_SIZE);

	for (uint8_t i = 0; i < qrs_det_rr_count; i++)
	{
		uint8_t ring_idx = (uint8_t)((rr_start_idx + i) % HRV_RR_BUFFER_SIZE);
		uint32_t rr_ms_u32 =
			(qrs_det_rr_intervals[ring_idx] * 1000u) / ECG_SAMPLE_RATE;
		if (rr_ms_u32 < min_rr_ms || rr_ms_u32 > max_rr_ms)
		{
			continue;
		}
		rr_ms_buf[rr_ms_count++] = (uint16_t)rr_ms_u32;
	}

	if (rr_ms_count < 2)
	{
		return 0;
	}

	uint16_t rr_sorted[HRV_RR_BUFFER_SIZE];
	for (uint8_t i = 0; i < rr_ms_count; i++)
	{
		rr_sorted[i] = rr_ms_buf[i];
	}

	for (uint8_t i = 1; i < rr_ms_count; i++)
	{
		uint16_t key = rr_sorted[i];
		int8_t j = (int8_t)(i - 1);
		while (j >= 0 && rr_sorted[j] > key)
		{
			rr_sorted[j + 1] = rr_sorted[j];
			j--;
		}
		rr_sorted[j + 1] = key;
	}

	uint16_t median;
	if ((rr_ms_count & 1u) != 0u)
	{
		median = rr_sorted[rr_ms_count / 2];
	}
	else
	{
		uint16_t a = rr_sorted[(rr_ms_count / 2) - 1];
		uint16_t b = rr_sorted[rr_ms_count / 2];
		median = (uint16_t)((a + b) / 2u);
	}

	uint16_t tol = (uint16_t)((median * 20u) / 100u);
	if (tol < 80u)
	{
		tol = 80u;
	}
	if (tol > 300u)
	{
		tol = 300u;
	}

	uint16_t rr_filt[HRV_RR_BUFFER_SIZE];
	uint8_t rr_filt_count = 0;
	for (uint8_t i = 0; i < rr_ms_count; i++)
	{
		uint16_t v = rr_ms_buf[i];
		uint16_t diff = (v > median) ? (uint16_t)(v - median) : (uint16_t)(median - v);
		if (diff <= tol)
		{
			rr_filt[rr_filt_count++] = v;
		}
	}

	if (rr_filt_count < 2)
	{
		for (uint8_t i = 0; i < rr_ms_count; i++)
		{
			rr_filt[i] = rr_ms_buf[i];
		}
		rr_filt_count = rr_ms_count;
	}

	uint64_t sum = 0;
	uint64_t sum_sq = 0;

	for (uint8_t i = 0; i < rr_filt_count; i++)
	{
		uint64_t v = rr_filt[i];
		sum += v;
		sum_sq += v * v;
	}

	uint64_t mean = sum / rr_filt_count;
	int64_t variance = (int64_t)(sum_sq / rr_filt_count) - (int64_t)(mean * mean);
	if (variance < 0)
	{
		variance = 0;
	}
	if ((uint64_t)variance > UINT32_MAX)
	{
		variance = UINT32_MAX;
	}

	uint32_t sdnn_u32 = ECG_Isqrt_U32((uint32_t)variance);
	if (sdnn_u32 > UINT16_MAX)
	{
		return UINT16_MAX;
	}
	return (uint16_t)sdnn_u32;
}

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

short CoeffBuf_40Hz_LowPass[FILTERORDER] = {
    1,    -6,   5,     2,    -12,   11,   8,     -26,  18,   20,    -49,  25,
    43,   -82,  28,    83,   -126,  22,   146,   -182, -2,   240,   -247, -56,
    377,  -317, -159,  573,  -387,  -343, 862,   -450, -679, 1341,  -501, -1407,
    2406, -535, -4188, 9276, 21299, 9276, -4188, -535, 2406, -1407, -501, 1341,
    -679, -450, 862,   -343, -387,  573,  -159,  -317, 377,  -56,   -247, 240,
    -2,   -182, 146,   22,   -126,  83,   28,    -82,  43,   25,    -49,  20,
    18,   -26,  8,     11,   -12,   2,    5,     -6,   1};

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

bool gUseSpi = true;
uint8_t gReadBuf[NUM_SAMPLES_PER_INT * NUM_BYTES_PER_SAMPLE * EXTRABUFFER] = {
    0};                                     // array to store register reads
bool gUseEcg = true;

// ============ 自适应增益控制变量 ============
static ecg_adaptive_gain_t adaptive_gain = {
    .enabled = true,                 // 默认启用自适应增益
    .current_gain = ECG_GAIN_LOW,    // 初始增益级别
    .signal_threshold_low = 5000,    // 信号幅度低于此值时增加增益
    .signal_threshold_high = 100000, // 信号幅度高于此值时降低增益
    .adjust_interval = 256,          // 每256个采样点(约2秒@128sps)评估一次
    .sample_count = 0,
    .peak_value = 0};

// Savitzky-Golay滤波器结构体和系数
typedef struct {
  short buffer[9];    // 9点缓冲区
  int index;          // 当前写入位置
  int is_buffer_full; // 缓冲区是否已填满
} sg_filter_t;

static const double sg_coeffs[9] = {-0.0909, 0.0606, 0.1688, 0.2338, 0.2554,
                                    0.2338,  0.1688, 0.0606, -0.0909};

static sg_filter_t sg_filter = {0};

// IIR/FIR 滤波器状态变量 (必须在 ECG_FilterState_Reset 之前定义)
static unsigned short ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1,
                      ECGFirstFlag = 1;
static short ECG_Pvev_DC_Sample = 0, ECG_Pvev_Sample = 0;

// 陷波滤波器重置标志
static bool notch_filter_reset_flag = false;

// 重置陷波滤波器状态
static void ECG_NotchFilter_Reset(void) { notch_filter_reset_flag = true; }

// 滤波器状态重置函数
static void ECG_FilterState_Reset(void) {
  int i;

  // 重置 IIR/FIR 滤波器状态
  ECGFirstFlag = 1; // 设置标志，下次调用 ECG_IIR_FIR_Filter 时会重新初始化
  ECG_bufStart = 0;
  ECG_bufCur = FILTERORDER - 1;
  ECG_Pvev_DC_Sample = 0;
  ECG_Pvev_Sample = 0;

  // 清空工作缓冲区
  for (i = 0; i < (2 * FILTERORDER); i++) {
    ECG_WorkingBuff[i] = 0;
  }

  // 重置 Savitzky-Golay 滤波器
  memset(&sg_filter, 0, sizeof(sg_filter));

  // 重置陷波滤波器状态
  ECG_NotchFilter_Reset();

  // 重置采样计数
  EcgSampleCount = 0;
}

static void ECG_Sleep_us(int us) { k_sleep(K_MSEC(1)); }

static void ECG_Sleep_ms(int ms) { k_sleep(K_MSEC(ms)); }

static void ECG_CS_LOW(void) { gpio_pin_set(gpio_0_ecg, ECG_CS_PIN, 0); }

static void ECG_CS_HIGH(void) { gpio_pin_set(gpio_0_ecg, ECG_CS_PIN, 1); }

void ECG_Int_Event(void) { ecg_int_flag = true; }

static void ECG_gpio_Init(void) {
  int err;
  gpio_flags_t flag = GPIO_INPUT;

  // 端口初始化
  gpio_0_ecg = DEVICE_DT_GET(ECG_PORT0);
  if (!gpio_0_ecg) {
    return;
  }
  gpio_1_ecg = DEVICE_DT_GET(ECG_PORT1);
  if (!gpio_1_ecg) {
    return;
  }
  // 设置为高，模式为测量ecg
  gpio_pin_configure(gpio_1_ecg, ECG_EN0_PIN, GPIO_OUTPUT);
  gpio_pin_set(gpio_1_ecg, ECG_EN0_PIN, 0);
  gpio_pin_configure(gpio_0_ecg, ECG_EN1_PIN, GPIO_OUTPUT);
  gpio_pin_set(gpio_0_ecg, ECG_EN1_PIN, 1);

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
  gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN,
                               GPIO_INT_ENABLE | GPIO_INT_EDGE_FALLING);
}

static void ECG_SPI_Init(void) {
  spi_ecg = DEVICE_DT_GET(ECG_DEV);
  if (!spi_ecg) {
    return;
  }

  spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
  spi_cfg.frequency = 4000000;
  spi_cfg.slave = 0;
}

static void ECG_SPI_Transceive(uint8_t *txbuf, uint32_t txbuflen,
                               uint8_t *rxbuf, uint32_t rxbuflen) {
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

  if (err) {
  }
}

void Max86176_WriteReg(uint8_t regAddr, uint8_t val) {
  uint8_t i = 0;
  uint8_t tx_buf[8];

  tx_buf[i++] = regAddr;
  tx_buf[i++] = 0x00;
  tx_buf[i++] = val;
  ECG_SPI_Transceive(tx_buf, i, NULL, 0);
}

void Max86176_ReadReg(uint8_t regAddr, uint8_t numBytes, uint8_t *readbuf) {
  uint8_t i = 0;
  uint8_t tx_buf[8];
  uint8_t rx_buf[numBytes + 2];

  tx_buf[i++] = regAddr;
  tx_buf[i++] = 0x80;
  ECG_SPI_Transceive(tx_buf, i, rx_buf, numBytes + 2);
  memcpy(readbuf, rx_buf + 2, numBytes);
}

void Max86176_StartEcg(void) {
  gUseEcg = true;
  Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 4); // 修改采样率为 128sps
  k_sleep(K_MSEC(50)); // ECG启动稳定延时 (200ms→50ms)
}

void Max86176_StopEcg(void) {
  gUseEcg = false;
  Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 4); // 修改采样率为128sps
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
	LOGD("Reg0 0x%02X, 0x%02X,0x%02X, 0x%02X,0x%02X, 0x%02X\n", readbuf[0], readbuf[1], readbuf[2], readbuf[3], readbuf[4], readbuf[5]);
#endif

	Max86176_WriteReg(0x10, 0);
#ifdef MAX86176_DEBUG
	LOGD("Max86176_Init");
#endif
	// Max86176_ReadReg(0x00, NUM_STATUS_REGS, gReadBuf);			  //
	// 连续读6个状态寄存器的值， read and clear all status registers
	Max86176_WriteReg(0x0d, AFE_FIFO_SIZE -	NUM_SAMPLES_PER_INT); // 设置FIFO数量，FIFO_A_FULL; assert
	// A_FULL on NUM_SAMPLES_PER_INT samples
	Max86176_WriteReg(0x80, 0x80); // 中断使能 A_FULL_EN; enable interrupt pin on A_FULL
	// assertion; ensure to enable the MCU's interrupt pin
	// Max86176_WriteReg(0x80, 0xE0);
	Max86176_WriteReg(0x1C, 0X20); // CLK_SEL; use internal 32.768kHz clock

  // PLL 配置: MDIV=0x1F, NDIV=0x13F (319)
  // PLL_CLK = FCLK * (MDIV + 288) / (NDIV + 1)
  // 32.768kHz * (31 + 288) / (319 + 1) = 32.768kHz * 319 / 320 ≈ 32.665kHz
  Max86176_WriteReg(0x19, 0x9F); // MDIV=0x1F, NDIV_MSB=1
  uint8_t s19 = 0;
  Max86176_ReadReg(0x19, 1, &s19);
#ifdef MAX86176_DEBUG
  LOGD("s19  : 0x%02X", s19);
#endif
  Max86176_WriteReg(0x1A, 0x3F); // NDIV_LSB=0x3F
  uint8_t s1a = 0;
  Max86176_ReadReg(0x1A, 1, &s1a);
#ifdef MAX86176_DEBUG
	LOGD("s1a  : 0x%02X", s1a);
#endif
  Max86176_WriteReg(0x18, 0x01); // PLL_EN=1

  // 等待 PLL 锁定，带超时保护（最大50次×100ms=5秒）
  uint8_t pll_status = 0;
  int pll_retry_count = 0;
  const int max_pll_retries = 50;

  Max86176_ReadReg(0x04, 1, &pll_status);
#ifdef MAX86176_DEBUG
  LOGD("wait pll lock begin");
#endif
  while (!(pll_status & 0x02) && (pll_retry_count < max_pll_retries)) {
#ifdef MAX86176_DEBUG
    LOGD("pll_status : 0x%02X, retry: %d", pll_status, pll_retry_count);
#endif
    ECG_Sleep_ms(100);
    Max86176_ReadReg(0x04, 1, &pll_status);
    pll_retry_count++;
  }

  if (pll_status & 0x02) {
#ifdef MAX86176_DEBUG
    LOGD("pll locked successfully after %d retries", pll_retry_count);
#endif
  } else {
    LOGD("pll lock timeout after %d retries!", max_pll_retries);
  }
  // ECG (32.768kHz*(0x1f+289)/(0x13f+1)/64=512Sps)
  // Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 2); //
  // 打开ECG，并且设置ECG的采样率 ECG_EN | ECG_DEC_RATE 采样频率为512sps
  // Max86176_WriteReg(0x90, ((gUseEcg ? 1 : 0) << 7) | 4); //
  // 修改采样率为128sps

  uint8_t ecg_config2 =
      (1 << 7) | (3 << 4) | (1);        // PGA=8x (011b), INA=20x (00b)
  Max86176_WriteReg(0x91, ecg_config2); // 设置ECG增益
  Max86176_WriteReg(
      0xa2,
      (0 << 7) | (0 << 6)); // ECG_P正极输入端和ECG_N负极输入端连接到ECG通道
  // OPEN_P | OPEN_N

  // ACLOFF (ADC: ~10Sps, DAC: 32.768kHz*(0x1f+289)/(10*64*(2+1))=5461.3Hz)
  // 0x93: bit7=EN_LON_DET, bit6-5=EN_LOFF_DET, bit4=DAC_STIM_MODE,
  // bit3-0=LOFF_SETTLE
  Max86176_WriteReg(
      0x93,
      (1 << 7) | (2 << 5) |
          15); // EN_LON_DET=1, EN_LOFF_DET=2,LOFF_SETTLE=15(最大稳定时间)
  Max86176_WriteReg(
      0x94, (1 << 6) | (2 << 4) |
                15); // HI_CM_RES_EN=1, LOFF_CG_MODE=2, LOFF_IMAG=15(最大 1.6uA)
  Max86176_WriteReg(0x95,
                    (1 << 7) | 2); // AC_LOFF_IWAVE=1(sine), AC_LOFF_FREQ_DIV=2
  Max86176_WriteReg(0x99, 0x10);   // AC_LOFF_THRESH
  Max86176_WriteReg(0x9a,
                    (0 << 6) | 5); // AC_LOFF_UTIL_PGA_GAIN | AC_LOFF_HPF
  // other relevant ACLOFF registers are at defaults: AC_LOFF_CONV=0 (~10Sps),
  // AC_LOFF_CMP=2 RLD
  Max86176_WriteReg(0xa8, (1 << 7) | (1 << 6) | (1 << 4) | (1 << 3) | (1 << 2) |
                              3); // RLD_EN | RLD_MODE | EN_RLD_OOR | ACTV_CM_P
                                  // // | ACTV_CM_N | RLD_GAIN
  Max86176_WriteReg(0xa9,
                    (0 << 7) | (1 << 6) | (0 << 4) |
                        0); // RLD_EXT_RES | SEL_VCM_IN | RLD_BW | BODY_BIAS_DAC

  // #ifdef MAX86176_DEBUG
  //   // ===== 导联检测函数测试死循环（调试用）=====
  //   // 先启动 ECG，LON/RLD 检测需要 ECG_EN=1 才能工作
  //   Max86176_WriteReg(0x90, (1 << 7) | 4); // ECG_EN=1, 128sps
  //   k_sleep(K_MSEC(500));                  // 等待 ECG 和导联检测电路稳定

  //   LOGD("=== Lead detect test loop start ===");
  //   while (1) {

  //     bool lead_on = Max86176_CheckLeadStatus();
  //     LOGD(" CheckLead=%d", lead_on);
  //     k_sleep(K_MSEC(1000));
  //   }
  //   // ===== 测试死循环结束 =====
  // #endif
}

#define NOTCH_B0 0.9910186348f
#define NOTCH_B1 1.5321355283f
#define NOTCH_B2 0.9910186348f
#define NOTCH_A1 1.5321355283f
#define NOTCH_A2 0.9820372695f

float ECG_NotchFilter(float input)
{
	static float x1 = 0, x2 = 0; // 过去两个输入
	static float y1 = 0, y2 = 0; // 过去两个输出
	float y;

  // 检查是否需要重置滤波器状态
  if (notch_filter_reset_flag) {
    x1 = 0;
    x2 = 0;
    y1 = 0;
    y2 = 0;
    notch_filter_reset_flag = false;
    return 0;
  }

  y = NOTCH_B0 * input + NOTCH_B1 * x1 + NOTCH_B2 * x2 - NOTCH_A1 * y1 -
      NOTCH_A2 * y2;

	// 更新状态
	x2 = x1;
	x1 = input;
	y2 = y1;
	y1 = y;

	return y;
}

/*
FIR滤波有限脉冲响应
*/
void ECG_FilterProcess(short *WorkingBuff, short *CoeffBuf, short *FilterOut)
{
	int i;
	int acc = 0;
	for (i = 0; i < FILTERORDER; i++)
	{
		acc += (*CoeffBuf++) * (*WorkingBuff--);
	}

	*FilterOut = (acc >> 15); // 根据滤波器精度调整右移位数
}

void ECG_IIR_FIR_Filter(short CurrAqsSample, short *FilteredOut)
{
	short *CoeffBuf;
	short temp1, temp2, ECGData;
	unsigned short Cur_Chan;
	short FiltOut;

  CoeffBuf = CoeffBuf_40Hz_LowPass;
  if (ECGFirstFlag) {
    // 使用第一个采样值预填充滤波器状态，消除启动瞬态响应
    // 这样滤波器从稳定状态开始工作，避免抛物线波形
    for (Cur_Chan = 0; Cur_Chan < (2 * FILTERORDER); Cur_Chan++) {
      ECG_WorkingBuff[Cur_Chan] =
          0; // FIR缓冲区初始化为0（因为去直流后信号围绕0）
    }
    // 预设IIR高通滤波器状态为当前采样值，使其立即稳定
    ECG_Pvev_DC_Sample = 0;          // 直流分量从0开始
    ECG_Pvev_Sample = CurrAqsSample; // 上一个采样值设为当前值
    ECGFirstFlag = 0;

    // 第一个采样点直接输出0（因为没有前一个差分值）
    FilteredOut[0] = 0;
    return;
  }

  temp1 = NRCOEFF * ECG_Pvev_DC_Sample; // First order IIR
  ECG_Pvev_DC_Sample = (CurrAqsSample - ECG_Pvev_Sample) + temp1;
  ECG_Pvev_Sample = CurrAqsSample;
  temp2 = ECG_Pvev_DC_Sample >> 2;
  ECGData = (long)temp2;
  ECG_WorkingBuff[ECG_bufCur] = ECGData;
  ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (short *)&FiltOut);
  ECG_WorkingBuff[ECG_bufStart] = ECGData;
  FilteredOut[0] = FiltOut;
  ECG_bufCur++;
  ECG_bufStart++;
  if (ECG_bufStart == (FILTERORDER - 1)) {
    ECG_bufStart = 0;
    ECG_bufCur = FILTERORDER - 1;
  }

	return;
}

short ECG_SavitzkyGolay_Filter(short new_sample) {
    // 将新样本存入缓冲区
    sg_filter.buffer[sg_filter.index] = new_sample;
    sg_filter.index++;
    
    // 循环缓冲区
    if (sg_filter.index >= 9) {
        sg_filter.index = 0;
        sg_filter.is_buffer_full = 1;
    }
    
    // 只有当缓冲区填满后才进行滤波
    if (!sg_filter.is_buffer_full) {
        return new_sample; // 缓冲区未满时返回原始值
    }
    
    // 计算卷积（滤波）
    double sum = 0.0;
    int coeff_index = 0;
    
    // 从当前索引向前遍历缓冲区（循环）
    for (int i = sg_filter.index, count = 0; count < 9; count++) {
        sum += sg_filter.buffer[i] * sg_coeffs[coeff_index++];
        
        i++;
        if (i >= 9) {
            i = 0; // 循环到缓冲区开头
        }
    }
    
    // 将结果转换回short类型，注意数据范围
  // int16_t result = (int16_t)(sum + 0.5); // 四舍五入
	int16_t result = (int16_t)(sum + (sum >= 0 ? 0.5f : -0.5f));
    
    return result;
}


short ECGFilteredData[4];

// ECG 数据批量发送缓冲区 (每 0.5 秒 64 个数据)
#define ECG_BATCH_SIZE 64
// 启动预热期：跳过前N个不稳定采样点
// 由于滤波器已采用状态预填充技术，预热期可以缩短至FIR阶数+SG滤波窗口
#define ECG_WARMUP_SAMPLES (FILTERORDER + 9) // FIR阶数(81) + SG窗口(9)

static bool ecg_warmup_done = false;
static int ecg_warmup_counter = 0;

static short ecg_batch_buffer[ECG_BATCH_SIZE];
static uint32_t ecg_batch_count = 0;

// 调试计数器
static uint32_t dbg_int_count = 0;        // 中断触发次数
static uint32_t dbg_work_count = 0;       // 工作队列执行次数
static uint32_t dbg_fifo_ready_count = 0; // FIFO准备好次数
static uint32_t dbg_consumer_count = 0;   // 消费者读取次数

// ============ 导联状态中断驱动检测（带防抖）============
static bool g_last_lead_status = false;    // 上一次确认的导联状态
static bool g_lead_status_inited = false;  // 导联状态是否已初始化
static bool g_lead_pending_status = false; // 待确认的新状态
static uint8_t g_lead_debounce_count = 0;  // 防抖计数器
#define LEAD_DEBOUNCE_THRESHOLD 4 // 连续4次一致才确认（约500ms@125ms/次）

// 使用k_work工作队列替代信号量+线程方案
static struct k_work afe_work;
static void afe_work_handler(struct k_work *work);

// 初始化工作队列
static bool afe_work_initialized = false;

static void afe_work_init(void) {
  if (!afe_work_initialized) {
    k_work_init(&afe_work, afe_work_handler);
    afe_work_initialized = true;
  }
}

// array to store ECG/ACLOFF/PPG ADC counts, time data
int32_t adcCountArr[NUM_ADC][NUM_SAMPLES_PER_INT *
                             EXTRABUFFER]; // array to store ECG/ACLOFF/PPG ADC
                                           // counts, time data

// 静态变量用于AFE处理
static int32_t gEcgSampleCount = -1;

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
  // ============ 读取状态寄存器 ============
  uint8_t status_regs[NUM_STATUS_REGS];
  Max86176_ReadReg(0x00, NUM_STATUS_REGS,
                   status_regs); // 读取并清除所有状态寄存器

  // ============
  // 被动检查导联状态（带防抖，借助A_FULL中断触发，每125ms检查一次）============
  {
    bool current_lead_on = Max86176_CheckLeadStatus();

    if (!g_lead_status_inited) {
      // 首次初始化：直接确认并上报
      g_last_lead_status = current_lead_on;
      g_lead_pending_status = current_lead_on;
      g_lead_debounce_count = 0;
      g_lead_status_inited = true;

      uint8_t buffer[32] = {0};
      if (current_lead_on) {
        strcpy(buffer, COM_ECG_LEAD_ON);
        MapcsSendData(UART_DATA_ECG, buffer, strlen(COM_ECG_LEAD_ON));
      } else {
        strcpy(buffer, COM_ECG_LEAD_OFF);
        MapcsSendData(UART_DATA_ECG, buffer, strlen(COM_ECG_LEAD_OFF));
      }

    } else if (current_lead_on != g_last_lead_status) {
      // 检测到与当前确认状态不同
      if (current_lead_on == g_lead_pending_status) {
        // 与上次待确认状态一致，累加防抖计数
        g_lead_debounce_count++;
      } else {
        // 新的待确认状态，重新开始计数
        g_lead_pending_status = current_lead_on;
        g_lead_debounce_count = 1;
      }

      // 达到防抖阈值，确认状态变化并上报
      if (g_lead_debounce_count >= LEAD_DEBOUNCE_THRESHOLD) {
        g_last_lead_status = current_lead_on;
        g_lead_debounce_count = 0;

        uint8_t buffer[32] = {0};
        if (current_lead_on) {
          strcpy(buffer, COM_ECG_LEAD_ON);
          MapcsSendData(UART_DATA_ECG, buffer, strlen(COM_ECG_LEAD_ON));
        } else {
          strcpy(buffer, COM_ECG_LEAD_OFF);
          MapcsSendData(UART_DATA_ECG, buffer, strlen(COM_ECG_LEAD_OFF));
        }
#ifdef MAX86176_DEBUG
        LOGD("Lead status changed: %s (debounced)",
             current_lead_on ? "ON" : "OFF");
#endif
      }
    } else {
      // 状态与已确认一致，重置防抖计数
      g_lead_debounce_count = 0;
    }
  }

  // 检查A_FULL标志位，处理FIFO数据
  if (!(status_regs[0] & 0x80))
    return; // 无FIFO数据需要处理

  // 循环处理直到FIFO为空或达到最大循环次数
  while (loop_count < max_loops) {
    // 第一次循环使用已读取的状态，后续循环重新读取
    if (loop_count > 0) {
      Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);
      if (!(status_regs[0] & 0x80))
        break; // FIFO为空，退出循环
    }

    // 重置每批次的sample索引
    memset(sampleIx, 0, sizeof(sampleIx));
    ecg_in_this_batch = 0;

    loop_count++;
    dbg_fifo_ready_count++;

    Max86176_ReadReg(0x0a, 2, gReadBuf); // read FIFO_DATA_COUNT
    uint32_t count =
        ((gReadBuf[0] & 0x80) << 1) |
        gReadBuf[1]; // FIFO_DATA_COUNT will be >= NUM_SAMPLES_PER_INT
    Max86176_ReadReg(0x0c, count * NUM_BYTES_PER_SAMPLE,
                     gReadBuf); // read FIFO_DATA
    for (readBufIx = 0; readBufIx < (count * NUM_BYTES_PER_SAMPLE);
         readBufIx += NUM_BYTES_PER_SAMPLE) // parse the FIFO data
    {
      tag = (gReadBuf[readBufIx] >> 4) & 0xf;
      if (tag == TAG_ECG) {
        data0 = gReadBuf[readBufIx];
        data1 = gReadBuf[readBufIx + 1];
        data2 = gReadBuf[readBufIx + 2];
        // ECG 数据是 18-bit 二补码
        ecg_value = ((data0 & 0x03) << 16) | (data1 << 8) | data2;

        // 直接在工作队列中处理ECG数据，不使用消费者线程
        // 18位二补码转换：如果bit17是1，则为负数
        int32_t ecg_signed = ecg_value;
        if (ecg_signed & 0x20000) {
          ecg_signed -= (1 << 18);
        }

        // 使用int32_t进行滤波，避免short溢出
        short curr_sample = (short)(ecg_signed);

        // 执行滤波
        ECG_IIR_FIR_Filter(curr_sample, &ECGFilteredData[1]);
        ECGFilteredData[1] = ECG_SavitzkyGolay_Filter(ECGFilteredData[1]);

        // ============ 启动预热期处理 ============
        if (!ecg_warmup_done) {
          ecg_warmup_counter++;
          if (ecg_warmup_counter < ECG_WARMUP_SAMPLES) {
            // 预热期内丢弃数据，不进行发送
            continue;
          } else {
            // 预热完成，重置批量缓冲区
            ecg_warmup_done = true;
            ecg_batch_count = 0;
#ifdef MAX86176_DEBUG
            LOGD("ECG warmup completed, %d samples discarded",
                 ecg_warmup_counter);
#endif
          }
        }
        // #ifdef MAX86176_DEBUG
        //         LOGD("ECG data %d", ECGFilteredData[1]);
        // #endif

        // 自适应增益处理 - 根据信号幅度动态调整增益
        Max86176_AdaptiveGainProcess(ecg_signed);
        // 将滤波后的数据存入批量缓冲区
        ecg_batch_buffer[ecg_batch_count] = ECGFilteredData[1];
        ecg_batch_count++;

        // 每收集 64 个数据 (0.5 秒) 后批量发送
        if (ecg_batch_count >= ECG_BATCH_SIZE) {
          uint32_t len;
          uint8_t buffer[ECG_BATCH_SIZE * 2 + 10] = {0};

#ifdef MAX86176_DEBUG
          // LOGD("Sending ECG data");
#endif
          strcpy(buffer, COM_ECG_GET_DATA);
          len = strlen(COM_ECG_GET_DATA);
          memcpy(&buffer[len], (void *)&ecg_batch_buffer, ECG_BATCH_SIZE * 2);
          MapcsSendData(UART_DATA_ECG, buffer, ECG_BATCH_SIZE * 2 + len);

          ecg_batch_count = 0;
        }

        ecg_in_this_batch++;

        gEcgSampleCount++;
        adcCountArr[IX_ECG][sampleIx[IX_ECG]++] =
            (gReadBuf[readBufIx + 0] >> 2) & 1;
        adcCountArr[IX_ECG][sampleIx[IX_ECG]] =
            ((gReadBuf[readBufIx + 0] & 0x3) << 16) +
            (gReadBuf[readBufIx + 1] << 8) + gReadBuf[readBufIx + 2];
        if (gReadBuf[readBufIx + 0] & 0x2)
          adcCountArr[IX_ECG][sampleIx[IX_ECG]] -= (1 << 18);

        sampleIx[IX_ECG]++;

      } else if (tag == TAG_LOFFUTIL) {
        tag = (gReadBuf[readBufIx + 0] >> 2) &
              1; // this can also be used for the array index in this example
        adcCountArr[tag][sampleIx[tag]] =
            ((gReadBuf[readBufIx + 1] & 0xf) << 8) + gReadBuf[readBufIx + 2];
        if (gReadBuf[readBufIx + 0] & 0x8)
          adcCountArr[tag][sampleIx[tag]] -= (1 << 12);

        sampleIx[tag]++;
        if (sampleIx[tag] >= NUM_SAMPLES_PER_INT * EXTRABUFFER) {
          sampleIx[tag] = 0;
        }
      }
    }
  }

  // 如果达到最大循环次数仍有数据，重新调度自己
  if (loop_count >= max_loops)
    k_work_submit(&afe_work);
}

void Max86176_onAfeInt(void) // call this on AFE interrupt
{
  dbg_int_count++;

  // 提交工作到系统工作队列，异步执行耗时操作
  k_work_submit(&afe_work);
}

void Max86176_Int_Disable(void)
{
	gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN, GPIO_INT_DISABLE);
}

static bool gpio_cb_added = false;  // 标记回调是否已添加
void Max86176_Int_Enable(void)
{
	gpio_pin_configure(gpio_1_ecg, ECG_INT_PIN, GPIO_INPUT);
	gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN, GPIO_INT_DISABLE);

	// 只在第一次添加回调，避免重复添加
	if(!gpio_cb_added)
	{
		gpio_init_callback(&gpio_cb, ECG_Int_Event, BIT(ECG_INT_PIN));
		gpio_add_callback(gpio_1_ecg, &gpio_cb);
		gpio_cb_added = true;
	}

  gpio_pin_interrupt_configure(gpio_1_ecg, ECG_INT_PIN,
                               GPIO_INT_ENABLE | GPIO_INT_EDGE_FALLING);
}

void Max86176_init(void)
{
	ECG_gpio_Init();
	ECG_SPI_Init();

	// 初始化AFE工作队列
	afe_work_init();

	k_sleep(K_MSEC(50));

	Sensor_Init();
	// 应用优化的RLD配置以增强抗干扰能力
	Max86176_OptimizeRLD();
	// 启用自适应增益控制
	Max86176_EnableAdaptiveGain(true);
}

void Max86176_start_measure(void)
{
  uint8_t fifo_cfg;
  uint8_t status_regs[NUM_STATUS_REGS];
  int flush_retry = 0;
  const int max_flush_retries = 3;

	// 先禁用中断，防止初始化过程中产生干扰
	Max86176_Int_Disable();

	// 清除所有未处理的中断标志和状态寄存器
	ecg_int_flag = false;
  Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);

  // 优化FIFO清除流程：多次刷新确保彻底清除残留数据
  for (flush_retry = 0; flush_retry < max_flush_retries; flush_retry++) {
    // 读取当前FIFO配置
    Max86176_ReadReg(0x0d, 1, &fifo_cfg);
    // 设置FIFO_FLUSH位(bit7)进行刷新
    Max86176_WriteReg(0x0d, fifo_cfg | 0x80);
    k_sleep(K_MSEC(10)); // 短暂等待刷新完成

    // 正确读取 FIFO_DATA_COUNT 寄存器 (0x0A-0x0B) 检查FIFO是否为空
    uint8_t fifo_count_regs[2];
    Max86176_ReadReg(0x0a, 2, fifo_count_regs);
    uint16_t fifo_count =
        ((fifo_count_regs[0] & 0x80) << 1) | fifo_count_regs[1];
    if (fifo_count == 0) {
      // FIFO已清空，退出循环
      break;
    }
  }
  // 清除所有状态寄存器中的中断标志
  Max86176_ReadReg(0x00, NUM_STATUS_REGS, status_regs);
  // 额外延迟确保FIFO完全稳定
  k_sleep(K_MSEC(20));

  // 重置批量发送计数器和相关状态
  ecg_batch_count = 0;
  gEcgSampleCount = 0;
  ecg_warmup_done = false; // 重置预热标志
  ecg_warmup_counter = 0;  // 重置预热计数器
  dbg_int_count = 0;
  dbg_work_count = 0;
  dbg_fifo_ready_count = 0;
  dbg_consumer_count = 0;

  // 重置滤波器状态 - 关键！防止旧数据影响新采集
  ECG_FilterState_Reset();

  // 重置自适应增益状态
  adaptive_gain.sample_count = 0;
  adaptive_gain.peak_value = 0;

  // 重置导联状态监测变量（含防抖）
  g_lead_status_inited = false;
  g_last_lead_status = false;
  g_lead_pending_status = false;
  g_lead_debounce_count = 0;

  // 重新配置中断使能寄存器
  Max86176_WriteReg(0x80, 0x80); // A_FULL_EN
  // LON状态在afe_work_handler中被动检查，不通过INT1触发

	// 重新启用 GPIO 中断
	Max86176_Int_Enable();

	// 启动 ECG 采集
	Max86176_StartEcg();

	// 短暂延迟让系统稳定
	k_sleep(K_MSEC(50));
}

void Max86176_stop(void)
{
	//先禁用中断，防止停止过程中产生干扰
	Max86176_Int_Disable();

	// 停止ECG采集
	Max86176_StopEcg();

	// 清除中断标志
	ecg_int_flag = false;

	// 清除状态寄存器
	Max86176_ReadReg(0x00, NUM_STATUS_REGS, gReadBuf);
	gpio_pin_configure(gpio_1_ecg, ECG_EN0_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_EN0_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_EN1_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_EN1_PIN, 0);

	gpio_pin_configure(gpio_1_ecg, ECG_I2C_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_1_ecg, ECG_I2C_CON_PIN, 0);
	gpio_pin_configure(gpio_0_ecg, ECG_SPI_CON_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_0_ecg, ECG_SPI_CON_PIN, 0);
}

void Max86176_start(void)
{
	uint8_t part_id;

  // 【关键】先禁用 GPIO 中断，防止芯片复位期间产生毛刺触发虚假中断
  Max86176_Int_Disable();
  ecg_int_flag = false;

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
  k_sleep(K_MSEC(100));

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

  // 【重要】先应用 RLD 优化配置，再启动测量，确保抗干扰配置在测量前生效
  Max86176_OptimizeRLD();

  // 启用自适应增益控制
  Max86176_EnableAdaptiveGain(true);

  k_sleep(K_MSEC(30)); // 等待 RLD 和增益配置生效 (50ms→30ms)

  // 再次清除可能在初始化期间产生的虚假中断标志
  ecg_int_flag = false;

  // 最后启动测量 (内部会重新启用中断)
  Max86176_start_measure();
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
	// INA_GAIN: 00=20x, 01=40x, 10=80x, 11=160x
	// PGA_GAIN固定为011b (8x)
	// ECG_IPOL = 1 (正常极性)
	uint8_t ina_gain = level;  // 直接映射: 0=20x, 1=40x, 2=80x, 3=160x
	uint8_t ecg_config2 = (1 << 7) | (3 << 4) | (ina_gain & 0x03);

	if(level > ECG_GAIN_VERY_HIGH)
		level = ECG_GAIN_VERY_HIGH;

	Max86176_WriteReg(0x91, ecg_config2);
	adaptive_gain.current_gain = level;

#ifdef MAX86176_DEBUG
	LOGD("ECG gain set to level %d (INA=%dx, total=%dx)", level, 20 << level, (20 << level) * 8);
#endif
}

/**
 * @brief 获取当前ECG增益级别
 */
ecg_gain_level_t Max86176_GetEcgGain(void) {
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

	adaptive_gain.sample_count++;

	// 达到评估间隔时进行增益调整判断
	if(adaptive_gain.sample_count >= adaptive_gain.adjust_interval)
	{
		ecg_gain_level_t new_gain = adaptive_gain.current_gain;

		// 信号太弱，需要增加增益
		if(adaptive_gain.peak_value < adaptive_gain.signal_threshold_low)
		{
			if(adaptive_gain.current_gain < ECG_GAIN_VERY_HIGH)
			{
				new_gain = adaptive_gain.current_gain + 1;
			#ifdef MAX86176_DEBUG	
				LOGD("ECG signal weak (peak=%d), increasing gain", adaptive_gain.peak_value);
			#endif
			}
		}
		// 信号太强(可能饱和)，需要降低增益
		else if(adaptive_gain.peak_value > adaptive_gain.signal_threshold_high)
		{
			if(adaptive_gain.current_gain > ECG_GAIN_LOW)
			{
				new_gain = adaptive_gain.current_gain - 1;
			#ifdef MAX86176_DEBUG	
				LOGD("ECG signal strong (peak=%d), decreasing gain", adaptive_gain.peak_value);
			#endif
			}
		}

		// 如果增益需要改变，则设置新增益
		if(new_gain != adaptive_gain.current_gain)
		{
			Max86176_SetEcgGain(new_gain);
		}

		// 重置计数器和峰值
		adaptive_gain.sample_count = 0;
		adaptive_gain.peak_value = 0;
	}
}

/**
 * @brief 优化RLD(右腿驱动)抗干扰配置
 * 
 * RLD用于减少共模干扰(如50/60Hz工频干扰)
 * 此函数配置最优的RLD参数以增强信号质量
 */
void Max86176_OptimizeRLD(void)
{
  // 先禁用RLD进行配置
  Max86176_WriteReg(0xa8, 0x00);
  k_sleep(K_MSEC(20));

  // 0xA8: RLD Configuration 1
  // bit7: RLD_EN = 1 (启用RLD)
  // bit6: RLD_MODE = 1 (使用内部反馈)
  // bit5: RLD_OOR_RAPID = 1 (快速检测RLD超出范围)
  // bit4: EN_RLD_OOR = 1 (启用RLD超出范围检测)
  // bit3: ACTV_CM_P = 1 (正极参与共模检测)
  // bit2: ACTV_CM_N = 1 (负极参与共模检测)
  // bit1-0: RLD_GAIN = 3 (最大增益，最强共模抑制)
  Max86176_WriteReg(0xa8, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 
                          (1 << 3) | (1 << 2) | 3);
  
  // 0xA9: RLD Configuration 2
  // bit7: RLD_EXT_RES = 0 (使用内部电阻)
  // bit6: SEL_VCM_IN = 1 (选择VCM输入)
  // bit5-4: RLD_BW = 2 (中等带宽，平衡响应速度和稳定性)
  // bit3-0: BODY_BIAS_DAC = 0 (无额外偏置)
  Max86176_WriteReg(0xa9, (0 << 7) | (1 << 6) | (2 << 4) | 0);
  
  // 0x94: Lead Detect Configuration 2 - 优化共模抑制
  // bit6: HI_CM_RES_EN = 1 (使用高共模输入阻抗，提高AC-LOFF精度)
  // bit5-4: LOFF_CG_MODE = 2 (使用RLD/lead bias模式)
  // bit3-0: LOFF_IMAG = 8 (400nA，增强导联检测灵敏度)
  Max86176_WriteReg(0x94, (1 << 6) | (2 << 4) | 8);

#ifdef MAX86176_DEBUG
  LOGD("RLD optimization applied for better noise rejection");
#endif
}

/**
 * @brief 检查导联连接状态
 * @return true - 导联连接良好; false - 导联脱落
 *
 * 读取 Status 5 (0x04) 寄存器的 RLD_IN_RANGE 位 (bit3):
 * - RLD_IN_RANGE=1: 导联连接良好
 * - RLD_IN_RANGE=0: 导联脱落
 */
bool Max86176_CheckLeadStatus(void) {
  uint8_t status5;

  Max86176_ReadReg(0x04, 1, &status5);

  bool rld_ok = (status5 >> 3) & 0x01;

#ifdef MAX86176_DEBUG
  static uint32_t log_counter = 0;
  // 每128次打印一次日志，避免刷屏（约1秒@128Hz采样）
  if (++log_counter >= 128) {
    LOGD("Status5=0x%02X, RLD_IN_RANGE=%d", status5, rld_ok);
    log_counter = 0;
  }
#endif

  return rld_ok;
}

void Max86176_Msg_Process(void)
{
	if(ecg_int_flag)
	{
	#ifdef MAX86176_DEBUG
		LOGD("ecg int!");
	#endif
		ecg_int_flag = false;
		Max86176_onAfeInt();
	}
}

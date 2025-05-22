#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <nrfx_qspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

LOG_MODULE_REGISTER(co5300_display, LOG_LEVEL_DBG);

// 屏幕分辨率定义
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// 引脚定义 - 基于nRF5340 DK开发板
#define CO5300_QSPI_BUS DT_NODELABEL(qspi)
#define CO5300_DC_PIN 29     // D/C引脚 (P0.24)
#define CO5300_RESET_PIN 28  // 复位引脚 (P0.28)
#define CO5300_BL_PIN 10     // 背光控制引脚 (P0.10)
#define CO5300_VCI_EN_PIN 23 // VCI使能引脚 (P0.23)
#define CO5300_CS_PIN 18     // 添加CS引脚定义

// CO5300命令集
#define CO5300_NOP 0x00
#define CO5300_SWRESET 0x01
#define CO5300_RDDID 0x04
#define CO5300_RDDST 0x09
#define CO5300_SLPIN 0x10
#define CO5300_SLPOUT 0x11
#define CO5300_PTLON 0x12
#define CO5300_NORON 0x13
#define CO5300_INVOFF 0x20
#define CO5300_INVON 0x21
#define CO5300_DISPOFF 0x28
#define CO5300_DISPON 0x29
#define CO5300_CASET 0x2A
#define CO5300_RASET 0x2B
#define CO5300_RAMWR 0x2C
#define CO5300_RAMRD 0x2E
#define CO5300_PTLAR 0x30
#define CO5300_VSCRDEF 0x33
#define CO5300_MADCTL 0x36
#define CO5300_VSCRSADD 0x37
#define CO5300_IDMOFF 0x38
#define CO5300_IDMON 0x39
#define CO5300_PIXSET 0x3A
#define CO5300_WRDISBV 0x51
#define CO5300_WRCTRLD 0x53
#define CO5300_WRCACE 0x55
#define CO5300_WRCABCMB 0x5E

// MADCTL位定义 - 显示方向控制
#define MADCTL_MY 0x80  // 行地址顺序
#define MADCTL_MX 0x40  // 列地址顺序
#define MADCTL_MV 0x20  // 行列交换
#define MADCTL_ML 0x10  // 垂直刷新顺序
#define MADCTL_RGB 0x00 // RGB颜色顺序
#define MADCTL_BGR 0x08 // BGR颜色顺序
#define MADCTL_MH 0x04  // 水平刷新顺序

// 像素格式定义
#define PIXFORMAT_16BPP 0x55 // RGB565 (16-bit)
#define PIXFORMAT_18BPP 0x66 // RGB666 (18-bit)
#define PIXFORMAT_24BPP 0x77 // RGB888 (24-bit)

// 颜色定义
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F

#define CO5300_R3A 0x3A // 像素格式设置（PIXSET）
#define CO5300_R35 0x35 // 垂直扫描起始地址设置（VSCRSADD）
#define CO5300_R53 0x53 // 显示控制寄存器（WRCTRLD）
#define CO5300_R51 0x51 // 显示亮度设置（WRDISBV）
#define CO5300_R63 0x63 // HBM显示亮度设置（WRHBMDISBV）
#define CO5300_R2A 0x2A // 列地址设置（CASET）
#define CO5300_R2B 0x2B // 行地址设置（RASET）
#define CO5300_R11 0x11 // 退出睡眠模式（SLPOUT）
#define CO5300_R29 0x29 // 开启显示（DISPON）

// 私有数据结构
static const struct device *qspi_dev = DEVICE_DT_GET(CO5300_QSPI_BUS);
static const struct device *gpio_dev;
static uint8_t dc_pin = CO5300_DC_PIN;
static uint8_t reset_pin = CO5300_RESET_PIN;
static uint8_t bl_pin = CO5300_BL_PIN;
static uint8_t vci_en_pin = CO5300_VCI_EN_PIN;
static uint8_t cs_pin = CO5300_CS_PIN;
// 帧缓冲区 - 存储屏幕像素数据
static uint16_t frame_buffer[SCREEN_WIDTH * SCREEN_HEIGHT];

// QSPI配置

#define SCREEN_PORT DT_NODELABEL(gpio0)
// 初始化控制引脚
static int init_control_pins(void)
{
    int ret;

    gpio_dev = DEVICE_DT_GET(SCREEN_PORT);
    if (!device_is_ready(gpio_dev))
    {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    // 配置DC引脚
    ret = gpio_pin_configure(gpio_dev, dc_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure DC pin: %d", ret);
        return ret;
    }

    // 配置复位引脚
    ret = gpio_pin_configure(gpio_dev, reset_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure RESET pin: %d", ret);
        return ret;
    }

    // 配置背光控制引脚
    // ret = gpio_pin_configure(gpio_dev, bl_pin, GPIO_OUTPUT_INACTIVE);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to configure BACKLIGHT pin: %d", ret);
    //     return ret;
    // }

    // 配置VCI使能引脚
    ret = gpio_pin_configure(gpio_dev, vci_en_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure VCI_EN pin: %d", ret);
        return ret;
    }

    ret = gpio_pin_configure(gpio_dev, cs_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure CS pin: %d", ret);
        return ret;
    }
    LOG_INF("gpio success");
    return 0;
}

// 设置DC引脚状态
static void set_dc_pin(bool state)
{
    gpio_pin_set(gpio_dev, dc_pin, state);
}

// 设置复位引脚状态
static void set_reset_pin(bool state)
{
    gpio_pin_set(gpio_dev, reset_pin, state);
}

// 设置背光引脚状态
static void set_backlight(bool state)
{
    gpio_pin_set(gpio_dev, bl_pin, state);
}

// 设置VCI使能引脚状态
static void set_vci_en(bool state)
{
    gpio_pin_set(gpio_dev, vci_en_pin, state);
}
static const nrf_qspi_cinstr_len_t cmd_cinstr_len[] = {
    [1] = NRF_QSPI_CINSTR_LEN_1B,
    [2] = NRF_QSPI_CINSTR_LEN_2B,
    [3] = NRF_QSPI_CINSTR_LEN_3B,
    [4] = NRF_QSPI_CINSTR_LEN_4B,
    [5] = NRF_QSPI_CINSTR_LEN_5B
    // 根据实际需求扩展
};

// 写寄存器函数（Command: 0x02）
static nrfx_err_t co5300_write_register(uint8_t reg_addr, const uint8_t *data, size_t len) {
    set_dc_pin(0);  // 命令模式
    gpio_pin_set(gpio_dev, cs_pin, 0);

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x02,          // 写寄存器指令码
        .length = NRF_QSPI_CINSTR_LEN_3B + len, // 3B指令 + 地址 + 数据
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false
    };

    uint8_t tx_buff[8] = {0};
    tx_buff[0] = reg_addr;      // 寄存器地址
    if (len > 0) {
        memcpy(tx_buff + 1, data, len);
    }

    nrfx_err_t res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, NULL);
    gpio_pin_set(gpio_dev, cs_pin, 1);
    return res;
}

// 读寄存器函数（Command: 0x03）
static nrfx_err_t co5300_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    set_dc_pin(0);  // 命令模式
    gpio_pin_set(gpio_dev, cs_pin, 0);

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x03,          // 读寄存器指令码
        .length = NRF_QSPI_CINSTR_LEN_3B + len, // 3B指令 + 地址 + 数据
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false
    };

    uint8_t tx_buff[4] = {reg_addr}; // 只需发送地址
    uint8_t rx_buff[8] = {0};

    nrfx_err_t res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, rx_buff);
    if (res == NRFX_SUCCESS) {
        memcpy(data, rx_buff + 1, len); // 数据从第二个字节开始
    }
    gpio_pin_set(gpio_dev, cs_pin, 1);
    return res;
}

static nrfx_err_t CO5300_write_command(uint8_t reg_addr, const uint8_t *data,
                                      size_t len)
{
    nrfx_err_t res;
    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x02, // 写寄存器指令码
        .length = NRF_QSPI_CINSTR_LEN_3B + len,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false};
    uint8_t tx_buff[1 + 3 + 7]; // 指令(1) + 地址(3) + 参数(最多7字节)

    // 构造QUAD SPI写命令格式
    tx_buff[0] = 0x02; // 指令字节
    tx_buff[1] = 0x00; // 地址字节1
    tx_buff[2] = reg_addr; // 地址字节2（命令字节）
    tx_buff[3] = 0x00; // 地址字节3
    if (len > 0)
    {
        memcpy(tx_buff + 4, data, len); // 参数数据
    }
    
    res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, NULL);
    return res;
}

// 发送命令到CO5300
void co5300_send_cmd(uint8_t cmd)
{
    // 设置为命令模式（DC引脚低电平）
    set_dc_pin(0);
    gpio_pin_set(gpio_dev, cs_pin, 0); // 选中设备
    nrf_qspi_cinstr_conf_t cfg = {
        .opcode = cmd,
        .length = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false,
    };

    nrfx_err_t err_code = nrfx_qspi_cinstr_xfer(&cfg, NULL, NULL);
    // 处理错误码
    if (err_code != NRFX_SUCCESS)
        LOG_ERR("QSPI write failed: Error code=0x%08X", err_code);
    else
        LOG_INF("QSPI write success");

    gpio_pin_set(gpio_dev, cs_pin, 1); // 取消选中
    k_sleep(K_MSEC(10));               // 短延时确保命令完成
}

// 发送命令和数据到CO5300
static void co5300_send_cmd_data(uint8_t reg_address, const uint8_t *data,
                                 size_t len)
{
    set_dc_pin(0);                     // 命令模式
    gpio_pin_set(gpio_dev, cs_pin, 0); // 拉低CS选中设备
    // 调用 CO5300_write_command 函数
    nrfx_err_t result = CO5300_write_command(reg_address, data, len);
    gpio_pin_set(gpio_dev, cs_pin, 1); // 拉高CS取消选中
}

// 发送多个字节到CO5300
static void co5300_send_data(const uint8_t *data, size_t len)
{
    // 设置为数据模式（DC引脚高电平）
    set_dc_pin(0);
    gpio_pin_set(gpio_dev, cs_pin, 0); // 选中设备

    // 分段传输大数据包（避免QSPI缓冲区溢出）
    nrf_qspi_cinstr_conf_t cfg = {
        .opcode = 0x02,
        .length = len,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false,
    };

    uint8_t tx_buff[8] = {0x00};
    //memcpy(tx_buff, data, len);
    nrfx_err_t err = nrfx_qspi_cinstr_xfer(&cfg, tx_buff, NULL);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Data transfer failed: 0x%08X", err);
    }
    else
    {
        LOG_INF("Data transfer success");
    }
    gpio_pin_set(gpio_dev, cs_pin, 1); // 取消选中
    k_sleep(K_MSEC(10));               // 保持与命令发送相同的延时
}

// 发送单个数据字节
static void co5300_send_data_byte(uint8_t data)
{
    co5300_send_data(&data, 1);
}

// 设置显示窗口
static void co5300_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t cmd_data[4];

    // 设置列地址
    co5300_send_cmd(CO5300_CASET);
    cmd_data[0] = (x0 >> 8) & 0xFF;
    cmd_data[1] = x0 & 0xFF;
    cmd_data[2] = (x1 >> 8) & 0xFF;
    cmd_data[3] = x1 & 0xFF;
    co5300_send_data(cmd_data, 4);

    // 设置行地址
    co5300_send_cmd(CO5300_RASET);
    cmd_data[0] = (y0 >> 8) & 0xFF;
    cmd_data[1] = y0 & 0xFF;
    cmd_data[2] = (y1 >> 8) & 0xFF;
    cmd_data[3] = y1 & 0xFF;
    co5300_send_data(cmd_data, 4);
}

// 初始化CO5300显示屏
int co5300_init(void)
{
    int ret;
    // 初始化控制引脚
    ret = init_control_pins();
    if (ret < 0)
    {
        LOG_ERR("Failed to initialize control pins: %d", ret);
        return ret;
    }
    qspi_init();
    // 启用VCI电源
    set_vci_en(1);
    k_sleep(K_MSEC(10)); // 等待VCI稳定
    // 硬件复位CO5300
    set_reset_pin(0);
    k_sleep(K_MSEC(100));
    set_reset_pin(1);
    k_sleep(K_MSEC(100));
    LOG_INF("start");
    // 软件复位
    // co5300_send_cmd(CO5300_SWRESET);
    // k_sleep(K_MSEC(150)); // 复位后需要较长时间等待

    // co5300_send_cmd(CO5300_NOP);

    // // RC4 80: SPI接口设置（假设对应MADCTL寄存器）
    // // 配置为SPI模式，RGB颜色顺序
    // co5300_send_cmd(CO5300_MADCTL);
    // co5300_send_data_byte(0x80 | MADCTL_RGB); // 假设RC4对应MADCTL的MY位

    // k_sleep(K_MSEC(15));
    // uint8_t madctl1 = co5300_read_reg(CO5300_R51);
    // LOG_INF("CO5300_R51 register value11111: 0x%02X", madctl1);
    // k_sleep(K_MSEC(15));
    // R3A 55: 设置像素格式为RGB565
    co5300_send_data_byte(PIXFORMAT_16BPP);
    // co5300_send_cmd_data(CO5300_R51, (uint8_t[]){0xFF}, 1);
    // uint8_t brightness;
    // co5300_read_register(CO5300_R51, &brightness, 1); // 应返回0xFF

    // LOG_INF("data is :%x", brightness);
    //   k_sleep(K_MSEC(15));
    //     uint8_t madctl2 = co5300_read_reg(CO5300_R51);
    //     LOG_INF("CO5300_R51 register value2222: 0x%02X", madctl2);

    // R35 00: 设置垂直滚动起始地址
    // co5300_send_cmd(CO5300_R35);
    // co5300_send_data_byte(0x00);

    // R53 20: 显示控制寄存器设置
    // co5300_send_cmd(CO5300_R53);
    // co5300_send_data_byte(0x20); // 具体功能需参考规格书

    // R51 FF: 设置显示亮度为最大
    // co5300_send_cmd(CO5300_R51);
    // co5300_send_data_byte(0xFF);

    // // R63 FF: 设置HBM模式亮度为最大
    // co5300_send_cmd(CO5300_R63);
    // co5300_send_data_byte(0xFF);

    // // R2A 00 00 01 3F: 设置列地址范围（0-319）
    // uint8_t ca_data[] = {0x00, 0x00, 0x01, 0x3F};  // 320像素
    // co5300_send_cmd(CO5300_R2A);
    // co5300_send_data(ca_data, sizeof(ca_data));

    // // R2B 00 00 00 EF: 设置行地址范围（0-239）
    // uint8_t ra_data[] = {0x00, 0x00, 0x00, 0xEF};  // 240像素
    // co5300_send_cmd(CO5300_R2B);
    // co5300_send_data(ra_data, sizeof(ra_data));

    // // R11: 退出睡眠模式
    // co5300_send_cmd(CO5300_R11);
    // k_sleep(K_MSEC(120));  // 退出睡眠后需要等待

    // // 延迟60ms
    // k_sleep(K_MSEC(60));

    // // R29: 开启显示
    // co5300_send_cmd(CO5300_R29);
    // LOG_INF("Display powered on");

    // // 清空屏幕为黑色
    // co5300_fill_screen(COLOR_BLACK);

    LOG_INF("CO5300 display initialized successfully");
    return 0;
}

// 填充整个屏幕为指定颜色
void co5300_fill_screen(uint16_t color)
{
    uint32_t i;
    uint8_t data[2] = {color >> 8, color & 0xFF};

    co5300_set_window(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    co5300_send_cmd(CO5300_RAMWR);

    // 发送大量数据时，分块发送以避免栈溢出
    for (i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
    {
        co5300_send_data(data, 2);
    }
}

// 在帧缓冲区中绘制像素
void co5300_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT)
    {
        return;
    }

    frame_buffer[y * SCREEN_WIDTH + x] = color;
}

// 将帧缓冲区内容刷新到屏幕
void co5300_flush(void)
{
    uint8_t pixel_data[2];
    int i, j;

    co5300_set_window(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    co5300_send_cmd(CO5300_RAMWR);

    // 将帧缓冲区数据发送到屏幕
    for (j = 0; j < SCREEN_HEIGHT; j++)
    {
        for (i = 0; i < SCREEN_WIDTH; i++)
        {
            uint16_t color = frame_buffer[j * SCREEN_WIDTH + i];
            pixel_data[0] = color >> 8;
            pixel_data[1] = color & 0xFF;
            co5300_send_data(pixel_data, 2);
        }
    }
}

// 在屏幕上绘制矩形
void co5300_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t x1, y1;

    for (x1 = x; x1 < x + w; x1++)
    {
        for (y1 = y; y1 < y + h; y1++)
        {
            co5300_draw_pixel(x1, y1, color);
        }
    }
}

// 在屏幕上绘制文本
void co5300_draw_text(int16_t x, int16_t y, const char *text, uint16_t color, uint16_t bg_color)
{
    // const uint8_t *font;
    // uint8_t c, i, j, line;

    // // 使用5x7像素的ASCII字体
    // font = (const uint8_t *)font_5x7;

    // while (*text) {
    //     c = *text++;
    //     if (c < 32 || c > 127) continue;

    //     c -= 32; // 调整到字体数据的偏移量

    //     for (i = 0; i < 5; i++) {
    //         line = font[c * 5 + i];
    //         for (j = 0; j < 7; j++) {
    //             if (line & (1 << j)) {
    //                 co5300_draw_pixel(x + i, y + j, color);
    //             } else if (bg_color != color) {
    //                 co5300_draw_pixel(x + i, y + j, bg_color);
    //             }
    //         }
    //     }

    //     x += 6; // 每个字符后留一个像素间距
    // }
}

// 5x7像素ASCII字体数据
static const uint8_t font_5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // 32: ' '
    0x00, 0x00, 0x5F, 0x00, 0x00, // 33: '!'
    0x00, 0x07, 0x00, 0x07, 0x00, // 34: '"'
    0x14, 0x7F, 0x14, 0x7F, 0x14, // 35: '#'
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // 36: '$'
    // 更多字体数据...
};

void qspi_init()
{
    nrfx_qspi_config_t config = {
        .pins = {
            .sck_pin = 17, // QSPI SCK 引脚
            .csn_pin = 18, // QSPI CS 引脚
            .io0_pin = 13, // QSPI IO0 引脚
            .io1_pin = 14, // QSPI IO1 引脚
            .io2_pin = 15, // QSPI IO2 引脚
            .io3_pin = 16  // QSPI IO3 引脚
        },
        .prot_if = {.readoc = NRF_QSPI_READOC_FASTREAD, .writeoc = NRF_QSPI_WRITEOC_PP, .addrmode = NRF_QSPI_ADDRMODE_24BIT, .dpmconfig = false},
        .phy_if = {.sck_freq = NRF_QSPI_FREQ_DIV4, .sck_delay = 0x20, .dpmen = false}};

    // Initialize the QSPI module
    nrfx_err_t err_code = nrfx_qspi_init(&config, NULL, NULL);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("QSPI initialization failed with error: %d\n", err_code);
    }
    else
    {
        LOG_INF("QSPI initialized successfully");
    }
}

// 主函数
void main(void)
{
    int ret;
    LOG_INF("Starting CO5300 display demo");
    ret = co5300_init();
    if (ret < 0)
    {
        LOG_ERR("CO5300 initialization failed: %d", ret);
        return;
    }

    // 清空屏幕
    // co5300_fill_screen(COLOR_BLACK);

    // // 在帧缓冲区绘制一些测试内容
    // co5300_draw_rect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, COLOR_WHITE);
    // co5300_draw_text(20, 30, "nRF5340 CO5300 Demo", COLOR_YELLOW, COLOR_BLACK);
    // co5300_draw_text(20, 50, "Hello, World!", COLOR_CYAN, COLOR_BLACK);

    // // 绘制一些彩色矩形
    // co5300_draw_rect(30, 80, 60, 40, COLOR_RED);
    // co5300_draw_rect(100, 80, 60, 40, COLOR_GREEN);
    // co5300_draw_rect(170, 80, 60, 40, COLOR_BLUE);
    // co5300_draw_rect(240, 80, 60, 40, COLOR_MAGENTA);

    // // 将帧缓冲区内容刷新到屏幕
    // co5300_flush();

    LOG_INF("Demo initialized successfully");

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
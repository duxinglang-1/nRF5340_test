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

// ��Ļ�ֱ��ʶ���
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// ���Ŷ��� - ����nRF5340 DK������
#define CO5300_QSPI_BUS DT_NODELABEL(qspi)
#define CO5300_DC_PIN 29     // D/C���� (P0.24)
#define CO5300_RESET_PIN 28  // ��λ���� (P0.28)
#define CO5300_BL_PIN 10     // ����������� (P0.10)
#define CO5300_VCI_EN_PIN 23 // VCIʹ������ (P0.23)
#define CO5300_CS_PIN 18     // ���CS���Ŷ���

// CO5300���
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

// MADCTLλ���� - ��ʾ�������
#define MADCTL_MY 0x80  // �е�ַ˳��
#define MADCTL_MX 0x40  // �е�ַ˳��
#define MADCTL_MV 0x20  // ���н���
#define MADCTL_ML 0x10  // ��ֱˢ��˳��
#define MADCTL_RGB 0x00 // RGB��ɫ˳��
#define MADCTL_BGR 0x08 // BGR��ɫ˳��
#define MADCTL_MH 0x04  // ˮƽˢ��˳��

// ���ظ�ʽ����
#define PIXFORMAT_16BPP 0x55 // RGB565 (16-bit)
#define PIXFORMAT_18BPP 0x66 // RGB666 (18-bit)
#define PIXFORMAT_24BPP 0x77 // RGB888 (24-bit)

// ��ɫ����
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F

#define CO5300_R3A 0x3A // ���ظ�ʽ���ã�PIXSET��
#define CO5300_R35 0x35 // ��ֱɨ����ʼ��ַ���ã�VSCRSADD��
#define CO5300_R53 0x53 // ��ʾ���ƼĴ�����WRCTRLD��
#define CO5300_R51 0x51 // ��ʾ�������ã�WRDISBV��
#define CO5300_R63 0x63 // HBM��ʾ�������ã�WRHBMDISBV��
#define CO5300_R2A 0x2A // �е�ַ���ã�CASET��
#define CO5300_R2B 0x2B // �е�ַ���ã�RASET��
#define CO5300_R11 0x11 // �˳�˯��ģʽ��SLPOUT��
#define CO5300_R29 0x29 // ������ʾ��DISPON��

// ˽�����ݽṹ
static const struct device *qspi_dev = DEVICE_DT_GET(CO5300_QSPI_BUS);
static const struct device *gpio_dev;
static uint8_t dc_pin = CO5300_DC_PIN;
static uint8_t reset_pin = CO5300_RESET_PIN;
static uint8_t bl_pin = CO5300_BL_PIN;
static uint8_t vci_en_pin = CO5300_VCI_EN_PIN;
static uint8_t cs_pin = CO5300_CS_PIN;
// ֡������ - �洢��Ļ��������
static uint16_t frame_buffer[SCREEN_WIDTH * SCREEN_HEIGHT];

// QSPI����

#define SCREEN_PORT DT_NODELABEL(gpio0)
// ��ʼ����������
static int init_control_pins(void)
{
    int ret;

    gpio_dev = DEVICE_DT_GET(SCREEN_PORT);
    if (!device_is_ready(gpio_dev))
    {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    // ����DC����
    ret = gpio_pin_configure(gpio_dev, dc_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure DC pin: %d", ret);
        return ret;
    }

    // ���ø�λ����
    ret = gpio_pin_configure(gpio_dev, reset_pin, GPIO_OUTPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure RESET pin: %d", ret);
        return ret;
    }

    // ���ñ����������
    // ret = gpio_pin_configure(gpio_dev, bl_pin, GPIO_OUTPUT_INACTIVE);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to configure BACKLIGHT pin: %d", ret);
    //     return ret;
    // }

    // ����VCIʹ������
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

// ����DC����״̬
static void set_dc_pin(bool state)
{
    gpio_pin_set(gpio_dev, dc_pin, state);
}

// ���ø�λ����״̬
static void set_reset_pin(bool state)
{
    gpio_pin_set(gpio_dev, reset_pin, state);
}

// ���ñ�������״̬
static void set_backlight(bool state)
{
    gpio_pin_set(gpio_dev, bl_pin, state);
}

// ����VCIʹ������״̬
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
    // ����ʵ��������չ
};

// д�Ĵ���������Command: 0x02��
static nrfx_err_t co5300_write_register(uint8_t reg_addr, const uint8_t *data, size_t len) {
    set_dc_pin(0);  // ����ģʽ
    gpio_pin_set(gpio_dev, cs_pin, 0);

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x02,          // д�Ĵ���ָ����
        .length = NRF_QSPI_CINSTR_LEN_3B + len, // 3Bָ�� + ��ַ + ����
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false
    };

    uint8_t tx_buff[8] = {0};
    tx_buff[0] = reg_addr;      // �Ĵ�����ַ
    if (len > 0) {
        memcpy(tx_buff + 1, data, len);
    }

    nrfx_err_t res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, NULL);
    gpio_pin_set(gpio_dev, cs_pin, 1);
    return res;
}

// ���Ĵ���������Command: 0x03��
static nrfx_err_t co5300_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    set_dc_pin(0);  // ����ģʽ
    gpio_pin_set(gpio_dev, cs_pin, 0);

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x03,          // ���Ĵ���ָ����
        .length = NRF_QSPI_CINSTR_LEN_3B + len, // 3Bָ�� + ��ַ + ����
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false
    };

    uint8_t tx_buff[4] = {reg_addr}; // ֻ�跢�͵�ַ
    uint8_t rx_buff[8] = {0};

    nrfx_err_t res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, rx_buff);
    if (res == NRFX_SUCCESS) {
        memcpy(data, rx_buff + 1, len); // ���ݴӵڶ����ֽڿ�ʼ
    }
    gpio_pin_set(gpio_dev, cs_pin, 1);
    return res;
}

static nrfx_err_t CO5300_write_command(uint8_t reg_addr, const uint8_t *data,
                                      size_t len)
{
    nrfx_err_t res;
    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x02, // д�Ĵ���ָ����
        .length = NRF_QSPI_CINSTR_LEN_3B + len,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false};
    uint8_t tx_buff[1 + 3 + 7]; // ָ��(1) + ��ַ(3) + ����(���7�ֽ�)

    // ����QUAD SPIд�����ʽ
    tx_buff[0] = 0x02; // ָ���ֽ�
    tx_buff[1] = 0x00; // ��ַ�ֽ�1
    tx_buff[2] = reg_addr; // ��ַ�ֽ�2�������ֽڣ�
    tx_buff[3] = 0x00; // ��ַ�ֽ�3
    if (len > 0)
    {
        memcpy(tx_buff + 4, data, len); // ��������
    }
    
    res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buff, NULL);
    return res;
}

// �������CO5300
void co5300_send_cmd(uint8_t cmd)
{
    // ����Ϊ����ģʽ��DC���ŵ͵�ƽ��
    set_dc_pin(0);
    gpio_pin_set(gpio_dev, cs_pin, 0); // ѡ���豸
    nrf_qspi_cinstr_conf_t cfg = {
        .opcode = cmd,
        .length = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = false,
    };

    nrfx_err_t err_code = nrfx_qspi_cinstr_xfer(&cfg, NULL, NULL);
    // ���������
    if (err_code != NRFX_SUCCESS)
        LOG_ERR("QSPI write failed: Error code=0x%08X", err_code);
    else
        LOG_INF("QSPI write success");

    gpio_pin_set(gpio_dev, cs_pin, 1); // ȡ��ѡ��
    k_sleep(K_MSEC(10));               // ����ʱȷ���������
}

// ������������ݵ�CO5300
static void co5300_send_cmd_data(uint8_t reg_address, const uint8_t *data,
                                 size_t len)
{
    set_dc_pin(0);                     // ����ģʽ
    gpio_pin_set(gpio_dev, cs_pin, 0); // ����CSѡ���豸
    // ���� CO5300_write_command ����
    nrfx_err_t result = CO5300_write_command(reg_address, data, len);
    gpio_pin_set(gpio_dev, cs_pin, 1); // ����CSȡ��ѡ��
}

// ���Ͷ���ֽڵ�CO5300
static void co5300_send_data(const uint8_t *data, size_t len)
{
    // ����Ϊ����ģʽ��DC���Ÿߵ�ƽ��
    set_dc_pin(0);
    gpio_pin_set(gpio_dev, cs_pin, 0); // ѡ���豸

    // �ֶδ�������ݰ�������QSPI�����������
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
    gpio_pin_set(gpio_dev, cs_pin, 1); // ȡ��ѡ��
    k_sleep(K_MSEC(10));               // �������������ͬ����ʱ
}

// ���͵��������ֽ�
static void co5300_send_data_byte(uint8_t data)
{
    co5300_send_data(&data, 1);
}

// ������ʾ����
static void co5300_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t cmd_data[4];

    // �����е�ַ
    co5300_send_cmd(CO5300_CASET);
    cmd_data[0] = (x0 >> 8) & 0xFF;
    cmd_data[1] = x0 & 0xFF;
    cmd_data[2] = (x1 >> 8) & 0xFF;
    cmd_data[3] = x1 & 0xFF;
    co5300_send_data(cmd_data, 4);

    // �����е�ַ
    co5300_send_cmd(CO5300_RASET);
    cmd_data[0] = (y0 >> 8) & 0xFF;
    cmd_data[1] = y0 & 0xFF;
    cmd_data[2] = (y1 >> 8) & 0xFF;
    cmd_data[3] = y1 & 0xFF;
    co5300_send_data(cmd_data, 4);
}

// ��ʼ��CO5300��ʾ��
int co5300_init(void)
{
    int ret;
    // ��ʼ����������
    ret = init_control_pins();
    if (ret < 0)
    {
        LOG_ERR("Failed to initialize control pins: %d", ret);
        return ret;
    }
    qspi_init();
    // ����VCI��Դ
    set_vci_en(1);
    k_sleep(K_MSEC(10)); // �ȴ�VCI�ȶ�
    // Ӳ����λCO5300
    set_reset_pin(0);
    k_sleep(K_MSEC(100));
    set_reset_pin(1);
    k_sleep(K_MSEC(100));
    LOG_INF("start");
    // �����λ
    // co5300_send_cmd(CO5300_SWRESET);
    // k_sleep(K_MSEC(150)); // ��λ����Ҫ�ϳ�ʱ��ȴ�

    // co5300_send_cmd(CO5300_NOP);

    // // RC4 80: SPI�ӿ����ã������ӦMADCTL�Ĵ�����
    // // ����ΪSPIģʽ��RGB��ɫ˳��
    // co5300_send_cmd(CO5300_MADCTL);
    // co5300_send_data_byte(0x80 | MADCTL_RGB); // ����RC4��ӦMADCTL��MYλ

    // k_sleep(K_MSEC(15));
    // uint8_t madctl1 = co5300_read_reg(CO5300_R51);
    // LOG_INF("CO5300_R51 register value11111: 0x%02X", madctl1);
    // k_sleep(K_MSEC(15));
    // R3A 55: �������ظ�ʽΪRGB565
    co5300_send_data_byte(PIXFORMAT_16BPP);
    // co5300_send_cmd_data(CO5300_R51, (uint8_t[]){0xFF}, 1);
    // uint8_t brightness;
    // co5300_read_register(CO5300_R51, &brightness, 1); // Ӧ����0xFF

    // LOG_INF("data is :%x", brightness);
    //   k_sleep(K_MSEC(15));
    //     uint8_t madctl2 = co5300_read_reg(CO5300_R51);
    //     LOG_INF("CO5300_R51 register value2222: 0x%02X", madctl2);

    // R35 00: ���ô�ֱ������ʼ��ַ
    // co5300_send_cmd(CO5300_R35);
    // co5300_send_data_byte(0x00);

    // R53 20: ��ʾ���ƼĴ�������
    // co5300_send_cmd(CO5300_R53);
    // co5300_send_data_byte(0x20); // ���幦����ο������

    // R51 FF: ������ʾ����Ϊ���
    // co5300_send_cmd(CO5300_R51);
    // co5300_send_data_byte(0xFF);

    // // R63 FF: ����HBMģʽ����Ϊ���
    // co5300_send_cmd(CO5300_R63);
    // co5300_send_data_byte(0xFF);

    // // R2A 00 00 01 3F: �����е�ַ��Χ��0-319��
    // uint8_t ca_data[] = {0x00, 0x00, 0x01, 0x3F};  // 320����
    // co5300_send_cmd(CO5300_R2A);
    // co5300_send_data(ca_data, sizeof(ca_data));

    // // R2B 00 00 00 EF: �����е�ַ��Χ��0-239��
    // uint8_t ra_data[] = {0x00, 0x00, 0x00, 0xEF};  // 240����
    // co5300_send_cmd(CO5300_R2B);
    // co5300_send_data(ra_data, sizeof(ra_data));

    // // R11: �˳�˯��ģʽ
    // co5300_send_cmd(CO5300_R11);
    // k_sleep(K_MSEC(120));  // �˳�˯�ߺ���Ҫ�ȴ�

    // // �ӳ�60ms
    // k_sleep(K_MSEC(60));

    // // R29: ������ʾ
    // co5300_send_cmd(CO5300_R29);
    // LOG_INF("Display powered on");

    // // �����ĻΪ��ɫ
    // co5300_fill_screen(COLOR_BLACK);

    LOG_INF("CO5300 display initialized successfully");
    return 0;
}

// ���������ĻΪָ����ɫ
void co5300_fill_screen(uint16_t color)
{
    uint32_t i;
    uint8_t data[2] = {color >> 8, color & 0xFF};

    co5300_set_window(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    co5300_send_cmd(CO5300_RAMWR);

    // ���ʹ�������ʱ���ֿ鷢���Ա���ջ���
    for (i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
    {
        co5300_send_data(data, 2);
    }
}

// ��֡�������л�������
void co5300_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT)
    {
        return;
    }

    frame_buffer[y * SCREEN_WIDTH + x] = color;
}

// ��֡����������ˢ�µ���Ļ
void co5300_flush(void)
{
    uint8_t pixel_data[2];
    int i, j;

    co5300_set_window(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    co5300_send_cmd(CO5300_RAMWR);

    // ��֡���������ݷ��͵���Ļ
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

// ����Ļ�ϻ��ƾ���
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

// ����Ļ�ϻ����ı�
void co5300_draw_text(int16_t x, int16_t y, const char *text, uint16_t color, uint16_t bg_color)
{
    // const uint8_t *font;
    // uint8_t c, i, j, line;

    // // ʹ��5x7���ص�ASCII����
    // font = (const uint8_t *)font_5x7;

    // while (*text) {
    //     c = *text++;
    //     if (c < 32 || c > 127) continue;

    //     c -= 32; // �������������ݵ�ƫ����

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

    //     x += 6; // ÿ���ַ�����һ�����ؼ��
    // }
}

// 5x7����ASCII��������
static const uint8_t font_5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // 32: ' '
    0x00, 0x00, 0x5F, 0x00, 0x00, // 33: '!'
    0x00, 0x07, 0x00, 0x07, 0x00, // 34: '"'
    0x14, 0x7F, 0x14, 0x7F, 0x14, // 35: '#'
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // 36: '$'
    // ������������...
};

void qspi_init()
{
    nrfx_qspi_config_t config = {
        .pins = {
            .sck_pin = 17, // QSPI SCK ����
            .csn_pin = 18, // QSPI CS ����
            .io0_pin = 13, // QSPI IO0 ����
            .io1_pin = 14, // QSPI IO1 ����
            .io2_pin = 15, // QSPI IO2 ����
            .io3_pin = 16  // QSPI IO3 ����
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

// ������
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

    // �����Ļ
    // co5300_fill_screen(COLOR_BLACK);

    // // ��֡����������һЩ��������
    // co5300_draw_rect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, COLOR_WHITE);
    // co5300_draw_text(20, 30, "nRF5340 CO5300 Demo", COLOR_YELLOW, COLOR_BLACK);
    // co5300_draw_text(20, 50, "Hello, World!", COLOR_CYAN, COLOR_BLACK);

    // // ����һЩ��ɫ����
    // co5300_draw_rect(30, 80, 60, 40, COLOR_RED);
    // co5300_draw_rect(100, 80, 60, 40, COLOR_GREEN);
    // co5300_draw_rect(170, 80, 60, 40, COLOR_BLUE);
    // co5300_draw_rect(240, 80, 60, 40, COLOR_MAGENTA);

    // // ��֡����������ˢ�µ���Ļ
    // co5300_flush();

    LOG_INF("Demo initialized successfully");

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
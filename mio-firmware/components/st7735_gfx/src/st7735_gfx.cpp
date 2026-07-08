#include "st7735_gfx.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ST7735_GFX";

// ── ST7735 command set ──────────────────────────────────────────────────────
#define ST7735_SWRESET  0x01
#define ST7735_SLPOUT   0x11
#define ST7735_INVOFF   0x20
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_MADCTL   0x36
#define ST7735_COLMOD   0x3A
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_GMCTRP1  0xE0
#define ST7735_GMCTRN1  0xE1
#define ST7735_NORON    0x13

ST7735_GFX::ST7735_GFX(const st7735_gfx_config_t &cfg)
    : Adafruit_GFX(cfg.width, cfg.height), _cfg(cfg)
{
}

void ST7735_GFX::writeCommand(uint8_t cmd)
{
    gpio_set_level((gpio_num_t)_cfg.pin_dc, 0); // command mode
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(_spi, &t);
}

void ST7735_GFX::writeData(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    gpio_set_level((gpio_num_t)_cfg.pin_dc, 1); // data mode
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(_spi, &t);
}

void ST7735_GFX::hwReset()
{
    if (_cfg.pin_rst < 0) return; // module has no reset pin wired
    gpio_set_level((gpio_num_t)_cfg.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)_cfg.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)_cfg.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

void ST7735_GFX::runInitSequence()
{
    writeCommand(ST7735_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));

    writeCommand(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(255));

    writeCommand(ST7735_FRMCTR1);
    { uint8_t d[] = {0x01, 0x2C, 0x2D}; writeData(d, sizeof(d)); }
    writeCommand(ST7735_FRMCTR2);
    { uint8_t d[] = {0x01, 0x2C, 0x2D}; writeData(d, sizeof(d)); }
    writeCommand(ST7735_FRMCTR3);
    { uint8_t d[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}; writeData(d, sizeof(d)); }

    writeCommand(ST7735_INVCTR);
    writeData8(0x07);

    writeCommand(ST7735_PWCTR1);
    { uint8_t d[] = {0xA2, 0x02, 0x84}; writeData(d, sizeof(d)); }
    writeCommand(ST7735_PWCTR2);
    writeData8(0xC5);
    writeCommand(ST7735_PWCTR3);
    { uint8_t d[] = {0x0A, 0x00}; writeData(d, sizeof(d)); }
    writeCommand(ST7735_PWCTR4);
    { uint8_t d[] = {0x8A, 0x2A}; writeData(d, sizeof(d)); }
    writeCommand(ST7735_PWCTR5);
    { uint8_t d[] = {0x8A, 0xEE}; writeData(d, sizeof(d)); }

    writeCommand(ST7735_VMCTR1);
    writeData8(0x0E);

    writeCommand(ST7735_INVOFF);

    writeCommand(ST7735_MADCTL);
    writeData8(0x60); // RGB order, row/col exchange for landscape-friendly default

    writeCommand(ST7735_COLMOD);
    writeData8(0x05); // 16 bits/pixel (RGB565)

    writeCommand(ST7735_GMCTRP1);
    { uint8_t d[] = {0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                      0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
      writeData(d, sizeof(d)); }
    writeCommand(ST7735_GMCTRN1);
    { uint8_t d[] = {0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                      0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
      writeData(d, sizeof(d)); }

    writeCommand(ST7735_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));

    writeCommand(ST7735_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool ST7735_GFX::begin()
{
    gpio_config_t dc_cfg = {};
    dc_cfg.pin_bit_mask = (1ULL << _cfg.pin_dc);
    dc_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&dc_cfg);

    if (_cfg.pin_rst >= 0) {
        gpio_config_t rst_cfg = {};
        rst_cfg.pin_bit_mask = (1ULL << _cfg.pin_rst);
        rst_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&rst_cfg);
    }

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = _cfg.pin_sclk;
    buscfg.mosi_io_num = _cfg.pin_mosi;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = _cfg.width * _cfg.height * 2; // full-frame RGB565 worst case

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 20 * 1000 * 1000; // 20MHz, comfortably within ST7735R spec
    devcfg.mode = 0;
    devcfg.spics_io_num = _cfg.pin_cs;
    devcfg.queue_size = 1;

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    hwReset();
    runInitSequence();

    ESP_LOGI(TAG, "ST7735 initialized (%dx%d)", _cfg.width, _cfg.height);
    return true;
}

void ST7735_GFX::setAddrWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    x0 += _cfg.xstart; x1 += _cfg.xstart;
    y0 += _cfg.ystart; y1 += _cfg.ystart;

    writeCommand(ST7735_CASET);
    { uint8_t d[] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
                      (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
      writeData(d, sizeof(d)); }

    writeCommand(ST7735_RASET);
    { uint8_t d[] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
                      (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};
      writeData(d, sizeof(d)); }

    writeCommand(ST7735_RAMWR);
}

void ST7735_GFX::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= _cfg.width || y >= _cfg.height) return;
    setAddrWindow(x, y, x, y);
    uint8_t d[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    writeData(d, 2);
}

void ST7735_GFX::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if (x >= _cfg.width || y >= _cfg.height || w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _cfg.width)  w = _cfg.width  - x;
    if (y + h > _cfg.height) h = _cfg.height - y;

    setAddrWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8, lo = color & 0xFF;
    // Push a row's worth of pixels at a time — bounded, stack-friendly buffer
    // instead of one giant malloc for the whole rect.
    uint8_t line[256]; // 128 pixels * 2 bytes, matches panel width headroom
    int16_t px_per_line = sizeof(line) / 2;
    for (int16_t i = 0; i < px_per_line && i < w; i++) {
        line[i * 2] = hi;
        line[i * 2 + 1] = lo;
    }

    gpio_set_level((gpio_num_t)_cfg.pin_dc, 1);
    int32_t remaining = (int32_t)w * h;
    while (remaining > 0) {
        int32_t chunk = remaining < px_per_line ? remaining : px_per_line;
        spi_transaction_t t = {};
        t.length = chunk * 2 * 8;
        t.tx_buffer = line;
        spi_device_polling_transmit(_spi, &t);
        remaining -= chunk;
    }
}

size_t ST7735_GFX::write(uint8_t c)
{
    Adafruit_GFX::write(c);
    return 1;
}

void ST7735_GFX::print(const char *text)
{
    for (const char *p = text; *p; ++p) {
        write((uint8_t)*p);
    }
}

void ST7735_GFX::println(const char *text)
{
    print(text);
    write('\n');
}

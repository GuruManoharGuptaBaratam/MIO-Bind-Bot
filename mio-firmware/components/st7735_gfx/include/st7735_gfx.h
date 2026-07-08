#pragma once

#include <Adafruit_GFX.h>
#include "driver/spi_master.h"

// RGB565 color constants for demo-state screens.
#define ST77_BLACK    0x0000
#define ST77_WHITE    0xFFFF
#define ST77_RED      0xF800
#define ST77_GREEN    0x07E0
#define ST77_BLUE     0x001F
#define ST77_YELLOW   0xFFE0
#define ST77_CYAN     0x07FF
#define ST77_ORANGE   0xFC00

typedef struct {
    int pin_sclk;
    int pin_mosi;
    int pin_cs;
    int pin_dc;
    int pin_rst;
    int width;   // panel native width  (128 for a 1.8" 128x160)
    int height;  // panel native height (160 for a 1.8" 128x160)
    // Some ST7735R modules (red-tab vs green-tab boards) need a small pixel
    // offset for the visible area to line up correctly. Start at 0/0 — if
    // the image appears shifted or clipped on one edge, this is the knob
    // to adjust, not a sign anything else is wrong.
    int xstart;
    int ystart;
} st7735_gfx_config_t;

class ST7735_GFX : public Adafruit_GFX {
public:
    explicit ST7735_GFX(const st7735_gfx_config_t &cfg);

    // Initializes SPI + sends the ST7735R power-on/init command sequence.
    // Must be called once before any drawing calls.
    bool begin();

    // Required override: Adafruit_GFX builds every other primitive
    // (lines, circles, text glyphs, etc.) out of repeated calls to this
    // unless a faster override exists for that specific shape.
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;

    // Fast-path override: pushes a whole rectangular block in one SPI
    // transaction instead of pixel-by-pixel. fillScreen()/background
    // clears in Adafruit_GFX are built on fillRect(), so this matters a
    // lot for a 128x160 panel.
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;

    // Print/println support — the ESP-IDF port of Adafruit_GFX requires
    // subclasses to re-expose these explicitly (see fasani/adafruit_gfx
    // README), they are not automatically usable otherwise.
    size_t write(uint8_t c) override;
    void print(const char *text);
    void println(const char *text);

private:
    st7735_gfx_config_t _cfg;
    spi_device_handle_t _spi = nullptr;

    void writeCommand(uint8_t cmd);
    void writeData(const uint8_t *data, size_t len);
    void writeData8(uint8_t val) { writeData(&val, 1); }
    void setAddrWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    void hwReset();
    void runInitSequence();
};

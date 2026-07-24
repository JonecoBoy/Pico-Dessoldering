#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUF_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
#define SSD1306_DEFAULT_I2C_ADDR 0x3C

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t buffer[SSD1306_BUF_SIZE];
} ssd1306_t;

// Initialize I2C hardware and send display initialization commands
bool ssd1306_init(ssd1306_t *disp, i2c_inst_t *i2c, uint8_t addr, uint sda_pin, uint scl_pin);

// Send command byte to display
void ssd1306_send_cmd(ssd1306_t *disp, uint8_t cmd);

// Clear framebuffer (fill with 0)
void ssd1306_clear(ssd1306_t *disp);

// Draw single pixel (color: true = pixel on, false = pixel off)
void ssd1306_draw_pixel(ssd1306_t *disp, int x, int y, bool color);

// Draw ASCII character at (x, y) coordinates
void ssd1306_draw_char(ssd1306_t *disp, int x, int y, char c);

// Draw null-terminated string at (x, y) coordinates
void ssd1306_draw_string(ssd1306_t *disp, int x, int y, const char *str);

// Flush framebuffer content to GM12864 / SSD1306 display via I2C
void ssd1306_show(ssd1306_t *disp);

#endif // SSD1306_H

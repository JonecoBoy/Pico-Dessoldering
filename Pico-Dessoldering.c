#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ssd1306.h"
#include <stdio.h>

// I2C Defines: i2c0 on GP4 (SDA) and GP5 (SCL)
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define BUZZER_PIN 15
#define BUTTON_PIN 16

int main() {
  stdio_init_all();

  // Initialize Buzzer on GPIO15 (Output, low by default)
  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);
  gpio_put(BUZZER_PIN, 0);

  // Initialize Button on GPIO16 (Input with internal Pull-Up, active LOW)
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);

  // Create display context object
  ssd1306_t disp;

  // Initialize display on i2c0 with auto-detection for GM12864 / ST7567 /
  // SSD1306
  bool disp_ok = ssd1306_init(&disp, I2C_PORT, SSD1306_DEFAULT_I2C_ADDR,
                              I2C_SDA_PIN, I2C_SCL_PIN);

  if (!disp_ok) {
    // Display not detected on I2C bus -> Warning: 2 short beeps
    for (int i = 0; i < 2; i++) {
      gpio_put(BUZZER_PIN, 1);
      sleep_ms(100);
      gpio_put(BUZZER_PIN, 0);
      sleep_ms(100);
    }
  } else {
    // Display successfully found & initialized -> 1 short beep
    gpio_put(BUZZER_PIN, 1);
    sleep_ms(80);
    gpio_put(BUZZER_PIN, 0);
  }

  while (true) {
    // Check if button is pressed (pulls GPIO16 to GND / level 0)
    bool button_pressed = (gpio_get(BUTTON_PIN) == 0);

    // Sound buzzer when button is pressed
    gpio_put(BUZZER_PIN, button_pressed);

    // Update OLED display content
    ssd1306_clear(&disp);

    // Draw top and bottom border lines
    for (int x = 0; x < SSD1306_WIDTH; x++) {
      ssd1306_draw_pixel(&disp, x, 0, true);
      ssd1306_draw_pixel(&disp, x, SSD1306_HEIGHT - 1, true);
    }

    // Render status text
    ssd1306_draw_string(&disp, 12, 6, "PICO DESSOLDERING");
    ssd1306_draw_string(&disp, 10, 24, "SDA:GP4  SCL:GP5");

    if (button_pressed) {
      ssd1306_draw_string(&disp, 10, 42, "TRIGGER: PRESSED");
    } else {
      ssd1306_draw_string(&disp, 10, 42, "TRIGGER: RELEASED");
    }

    // Flush frame buffer to OLED display
    ssd1306_show(&disp);

    sleep_ms(30);
  }

  return 0;
}

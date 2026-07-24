#include "hardware/adc.h"
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
#define BUTTON2_PIN 1

// ADC2 for Pistol Sensor on GPIO28 (Pin 34)
#define SENSOR_ADC_PIN 28
#define SENSOR_ADC_CHAN 2

int main() {
  stdio_init_all();

  // Initialize ADC on GPIO28 (ADC2 - Pistol Sensor)
  adc_init();
  adc_gpio_init(SENSOR_ADC_PIN);
  adc_select_input(SENSOR_ADC_CHAN);

  // Initialize Buzzer on GPIO15 (Output, low by default)
  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);
  gpio_put(BUZZER_PIN, 0);

  // Initialize Button inputs on GPIO16 and GPIO1 (Internal Pull-Up, active LOW)
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);

  gpio_init(BUTTON2_PIN);
  gpio_set_dir(BUTTON2_PIN, GPIO_IN);
  gpio_pull_up(BUTTON2_PIN);

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
    // Read raw digital values from button pins (1 = released, 0 = pressed to GND)
    uint8_t val16 = gpio_get(BUTTON_PIN);
    uint8_t val1 = gpio_get(BUTTON2_PIN);

    // Button is pressed if either pin is pulled LOW (0)
    bool button_pressed = (val16 == 0) || (val1 == 0);

    // Read ADC2 (Pistol Sensor on GPIO28)
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw * (3.3f / 4095.0f);

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

    // Display ADC value and converted voltage on second line
    char adc_str[24];
    snprintf(adc_str, sizeof(adc_str), "ADC: %4d (%1.2fV)", adc_raw, voltage);
    ssd1306_draw_string(&disp, 10, 24, adc_str);

    // Render trigger status and raw pin values
    char trg_str[28];
    snprintf(trg_str, sizeof(trg_str), "TRG:%s GP16:%d GP1:%d",
             button_pressed ? "ON " : "OFF", val16, val1);
    ssd1306_draw_string(&disp, 6, 42, trg_str);

    // Flush frame buffer to OLED display
    ssd1306_show(&disp);

    sleep_ms(30);
  }

  return 0;
}

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "ssd1306.h"
#include <stdio.h>

static const float SUPPLY_VOLTAGE = 3.3f;
static const float R4_PULLUP = 75.0f;
static const float R3_GROUND = 75.0f;

static float adc_raw_to_voltage(uint16_t raw) {
  return raw * (SUPPLY_VOLTAGE / 4095.0f);
}

static float adc_voltage_to_parallel_resistance(float voltage) {
  if (voltage <= 0.0f || voltage >= SUPPLY_VOLTAGE) {
    return -1.0f;
  }
  return R4_PULLUP * voltage / (SUPPLY_VOLTAGE - voltage);
}

static float parallel_to_sensor_resistance(float rp) {
  if (rp <= 0.0f || rp >= R3_GROUND) {
    return -1.0f;
  }
  return (R3_GROUND * rp) / (R3_GROUND - rp);
}

// --- ADC -> Celsius lookup table (measured points)
// Arrays must be ascending in ADC value
static const uint16_t adc_table[] = {
    1040, 1090, 1120, 1150, 1200, 1220, 1240, 1255, 1270, 1300,
    1315, 1330, 1360, 1380, 1410, 1480, 1535, 1620, 1660, 1680,
    1700, 1720, 1760, 1780, 1790, 1820, 1880, 1900, 1950, 2000,
    2070, 2090, 2150, 2175, 2200, 2280
};

static const float temp_table[] = {
    25.0f, 30.0f, 36.0f, 40.0f, 51.0f, 57.0f, 60.0f, 61.0f, 64.0f, 73.0f,
    75.0f, 79.0f, 90.0f, 95.5f, 104.0f, 118.0f, 138.0f, 152.0f, 162.0f, 171.0f,
    176.0f, 180.0f, 191.0f, 198.0f, 202.0f, 212.0f, 231.0f, 246.0f, 250.0f, 282.0f,
    300.0f, 310.0f, 320.0f, 332.0f, 346.0f, 380.0f
};

static int adc_to_celsius(uint16_t adc_raw) {
  const int n = sizeof(adc_table) / sizeof(adc_table[0]);
  if (n < 2) return -1000;
  if (adc_raw <= adc_table[0]) return (int)(temp_table[0] + 0.5f);
  if (adc_raw >= adc_table[n-1]) return (int)(temp_table[n-1] + 0.5f);
  for (int i = 0; i < n - 1; ++i) {
    uint16_t a0 = adc_table[i];
    uint16_t a1 = adc_table[i+1];
    if (adc_raw >= a0 && adc_raw <= a1) {
      float t0 = temp_table[i];
      float t1 = temp_table[i+1];
      float ratio = (float)(adc_raw - a0) / (float)(a1 - a0);
      float t = t0 + ratio * (t1 - t0);
      return (int)(t + 0.5f);
    }
  }
  return -1000;
}

// I2C Defines: i2c0 on GP4 (SDA) and GP5 (SCL)
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define BUZZER_PIN 15
#define PUMP_PIN 3
#define IRON_PIN 2
#define BUTTON_PIN 16
#define BUTTON2_PIN 1

#define SENSOR_ADC_PIN 28
#define SENSOR_ADC_CHAN 2

#define KNOB_ADC_PIN 26
#define KNOB_ADC_CHAN 0

// Knob -> setpoint configuration
static const uint16_t KNOB_ADC_MIN = 50;    // knob min reading ~50 -> max temperature
static const uint16_t KNOB_ADC_MAX = 4095;  // knob max reading -> min temperature
static const int SET_TEMP_MIN = 30;         // degrees C at knob max
static const int SET_TEMP_MAX = 400;        // degrees C at knob min
static const int SET_TEMP_STEP = 5;         // step size in degrees

static int knob_adc_to_setpoint(uint16_t adc) {
  if (adc >= KNOB_ADC_MAX) return SET_TEMP_MIN;
  if (adc <= KNOB_ADC_MIN) return SET_TEMP_MAX;
  float ratio = (float)(adc - KNOB_ADC_MIN) / (float)(KNOB_ADC_MAX - KNOB_ADC_MIN);
  float inv = 1.0f - ratio; // invert so high ADC -> low temp
  float tempf = SET_TEMP_MIN + inv * (SET_TEMP_MAX - SET_TEMP_MIN);
  // quantize to nearest SET_TEMP_STEP
  int tempq = (int)((tempf + (SET_TEMP_STEP/2.0f)) / SET_TEMP_STEP) * SET_TEMP_STEP;
  if (tempq < SET_TEMP_MIN) tempq = SET_TEMP_MIN;
  if (tempq > SET_TEMP_MAX) tempq = SET_TEMP_MAX;
  return tempq;
}

int main() {
  stdio_init_all();

  // Initialize ADC on GPIO28 (ADC2 - Pistol Sensor)
  adc_init();
  adc_gpio_init(SENSOR_ADC_PIN);
  adc_select_input(SENSOR_ADC_CHAN);

  // Initialize ADC on GPIO26 (ADC0 - Knob)
  adc_gpio_init(KNOB_ADC_PIN);
  adc_select_input(KNOB_ADC_CHAN);

  // Initialize Buzzer on GPIO15 (Output, low by default)
  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);
  gpio_put(BUZZER_PIN, 0);

  gpio_init(PUMP_PIN);
  gpio_set_dir(PUMP_PIN, GPIO_OUT);
  gpio_put(PUMP_PIN, 0);

  gpio_init(IRON_PIN);
  gpio_set_dir(IRON_PIN, GPIO_OUT);
  gpio_put(IRON_PIN, 0);


  // Initialize Button inputs on GPIO16 and GPIO1 (Internal Pull-Up, active LOW)
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);

  gpio_init(BUTTON2_PIN);
  gpio_set_dir(BUTTON2_PIN, GPIO_IN);
  gpio_pull_up(BUTTON2_PIN);

  // Create display context object
  ssd1306_t disp;

  //  Initialize display on i2c0 with auto-detection for GM12864 / ST7567 /
  //  SSD1306
  bool disp_ok = ssd1306_init(&disp, I2C_PORT, SSD1306_DEFAULT_I2C_ADDR,
                              I2C_SDA_PIN, I2C_SCL_PIN);
  bool display_ready = disp_ok;

  if (display_ready) {
    // Clear screen
    ssd1306_clear(&disp);
    // Draw static border
    for (int x = 0; x < SSD1306_WIDTH; x++) {
      ssd1306_draw_pixel(&disp, x, 0, true);
      ssd1306_draw_pixel(&disp, x, SSD1306_HEIGHT - 1, true);
    }
    // Render static title
    ssd1306_draw_string(&disp, 12, 6, "PICO DESSOLDERING");
    // Flush once
    ssd1306_show(&disp);
  }

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

  // Setup non-blocking display refresh timing (1 second)
  const uint64_t refresh_interval_us = 1000000u; // 1s
  uint64_t next_refresh_us = time_us_64() + refresh_interval_us;

  while (true) {
    // Read raw digital values from button pins (1 = released, 0 = pressed to
    // GND)
    uint8_t val16 = gpio_get(BUTTON_PIN);
    uint8_t val1 = gpio_get(BUTTON2_PIN);

    // Button is pressed if either pin is pulled LOW (0)
    bool button_pressed = (val16 == 0) || (val1 == 0);

    // Select sensor channel and read its value
    adc_select_input(SENSOR_ADC_CHAN);
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw_to_voltage(adc_raw);
    float sensor_r = -1.0f;
    bool sensor_valid = false;

    // compute equivalent parallel resistance and sensor resistance
    float rp = adc_voltage_to_parallel_resistance(voltage);
    if (rp > 0.0f && rp < R3_GROUND) {
      sensor_r = parallel_to_sensor_resistance(rp);
      sensor_valid = sensor_r > 0.0f;
    }

    // Convert raw ADC to Celsius using measured calibration table
    int temp_c = adc_to_celsius(adc_raw);
    bool temp_valid = (temp_c > -100);

    // Select knob channel and read its value
    adc_select_input(KNOB_ADC_CHAN);
    uint16_t adc_knob = adc_read();
    float voltage_knob = adc_knob * (3.3f / 4095.0f);

    // Sound buzzer and activate pump when button is pressed
    // gpio_put(BUZZER_PIN, button_pressed);
    // gpio_put(PUMP_PIN, button_pressed);
    gpio_put(IRON_PIN, !button_pressed);

    // Update only dynamic parts of the display on the refresh interval
    if (display_ready) {
      uint64_t now_us = time_us_64();
      if (now_us >= next_refresh_us) {
        next_refresh_us += refresh_interval_us;
        // Clear dynamic area (keep border and title)
        // Simple method: clear entire then redraw static border and title again
        // (fast enough)
        ssd1306_clear(&disp);
        // Redraw static border
        for (int x = 0; x < SSD1306_WIDTH; x++) {
          ssd1306_draw_pixel(&disp, x, 0, true);
          ssd1306_draw_pixel(&disp, x, SSD1306_HEIGHT - 1, true);
        }
        // Title again
        ssd1306_draw_string(&disp, 12, 4, "PICO DESSOLDERING");

        // Dynamic THR (ADC) value with temperature
        char adc_str[32];
        if (temp_valid && temp_c >= 0) {
          snprintf(adc_str, sizeof(adc_str), "THR: %4d (%3dC)", adc_raw, temp_c);
        } else {
          snprintf(adc_str, sizeof(adc_str), "THR: %4d (---C)", adc_raw);
        }
        ssd1306_draw_string(&disp, 10, 14, adc_str);

        // Dynamic sensor resistance value
        char res_str[32];
        if (sensor_valid) {
          snprintf(res_str, sizeof(res_str), "R: %5.1f ohm", sensor_r);
        } else {
          snprintf(res_str, sizeof(res_str), "R: --- ohm");
        }
        ssd1306_draw_string(&disp, 10, 26, res_str);

        // Dynamic KNB value + setpoint
        int set_temp = knob_adc_to_setpoint(adc_knob);
        char adc_str2[48];
        snprintf(adc_str2, sizeof(adc_str2), "KNB:%4d SET:%3dC", adc_knob, set_temp);
        ssd1306_draw_string(&disp, 6, 38, adc_str2);

        // Trigger status
        char trg_str[28];
        snprintf(trg_str, sizeof(trg_str), "TRG:%s GP16:%d GP1:%d",
           button_pressed ? "ON " : "OFF", val16, val1);
        ssd1306_draw_string(&disp, 6, 50, trg_str);
        // Show updated frame
        ssd1306_show(&disp);
      }
    }
    // Reset watchdog to avoid reset (if enabled)
    // (Assumes watchdog has been init elsewhere)
    // watchdog_update();
    // Increase sleep to reduce I2C traffic
    sleep_ms(150);
  }

  return 0;
}

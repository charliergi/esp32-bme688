#pragma once

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "bsec2.h"

#define BSEC_TAG "BSEC_APP"

#define PANIC_LED   GPIO_NUM_2   // adjust for your board
#define PIN_CS      GPIO_NUM_10
#define SAMPLE_RATE BSEC_SAMPLE_RATE_LP
#define TAG "BSEC_APP"

// Function declarations with extern to avoid multiple definitions
int8_t bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme68x_delay_us(uint32_t period, void *intf_ptr);
unsigned long get_millis();
// void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec);
void checkBsecStatus(Bsec2 bsec);
void errLeds();

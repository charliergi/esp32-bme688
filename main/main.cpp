/**
 * @file main.cpp
 * @brief ESP-IDF adaptation of the basic.ino BSEC2 example.
 *
 * This file is designed to be the main entry point for an ESP-IDF project.
 * It replaces Arduino-specific functions with their ESP-IDF equivalents for
 * logging, GPIO control, and SPI communication.
 *
 * Copyright (C) 2021 Bosch Sensortec GmbH
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Modifications for ESP-IDF Copyright (C) 2025
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // For ets_delay_us
#include "esp_timer.h"   // For esp_timer_get_time

// Include the BSEC2 library header
#include "bsec2.h"

// --- Configuration ---
static const char* TAG = "BSEC_EXAMPLE";

// GPIO settings for ESP32-S3 (please verify for your specific board)
#define PANIC_LED   GPIO_NUM_2      // Example: Built-in LED on many ESP32-S3 boards
#define PIN_SCK     GPIO_NUM_12
#define PIN_MISO    GPIO_NUM_13
#define PIN_MOSI    GPIO_NUM_11
#define PIN_CS      GPIO_NUM_10

// SPI Host
#define BME_SPI_HOST SPI2_HOST

// BSEC constants from original .ino
#define ERROR_DUR   1000
#define SAMPLE_RATE BSEC_SAMPLE_RATE_LP

// --- Global Objects and Variables ---
Bsec2 envSensor; // Create an object of the class Bsec2
spi_device_handle_t spi_handle; // Handle for the SPI device

// --- Helper function declarations ---
void errLeds(void);
void checkBsecStatus(Bsec2 bsec);
void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec);

// --- ESP-IDF specific BME68x interface functions ---

/**
 * @brief SPI write function for ESP-IDF
 */
int8_t bme_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t handle = *(spi_device_handle_t *)intf_ptr;
    esp_err_t ret;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA; // Use tx_data for single-byte address
    t.tx_data[0] = reg_addr & 0x7F; // Write operation: MSB is 0
    t.length = 8; // 8 bits for the register address

    // Transmit the register address
    ret = spi_device_polling_transmit(handle, &t);
    if (ret != ESP_OK) return -1;

    // Transmit the data
    if (len > 0) {
        memset(&t, 0, sizeof(t));
        t.tx_buffer = reg_data;
        t.length = len * 8; // length is in bits
        ret = spi_device_polling_transmit(handle, &t);
        if (ret != ESP_OK) return -1;
    }
    
    return 0;
}

/**
 * @brief SPI read function for ESP-IDF
 */
int8_t bme_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t handle = *(spi_device_handle_t *)intf_ptr;
    esp_err_t ret;

    if (len == 0) {
        return 0;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8; // Total bits to receive
    t.rxlength = len * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = reg_data;
    t.cmd = reg_addr | 0x80; // Command phase: register address with read bit

    ret = spi_device_polling_transmit(handle, &t);

    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief Delay function wrapper for BSEC library
 */
void bme_delay_us(uint32_t period, void *intf_ptr) {
    ets_delay_us(period);
}

/**
 * @brief Milliseconds getter function for BSEC library
 */
unsigned long bme_millis() {
    return (unsigned long) (esp_timer_get_time() / 1000);
}


// --- Main Application Entry Point ---
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting BSEC2 Example for ESP-IDF");

    // --- GPIO & SPI Bus Initialization ---
    
    // Configure Panic LED
    gpio_reset_pin(PANIC_LED);
    gpio_set_direction(PANIC_LED, GPIO_MODE_OUTPUT);

    // Configure SPI bus
    spi_bus_config_t buscfg = {}; // Zero-initialize
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(BME_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configure SPI device
    spi_device_interface_config_t devcfg = {}; // Zero-initialize
    devcfg.mode = 0; // SPI mode 0
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
    devcfg.spics_io_num = PIN_CS; // Chip select pin 
    devcfg.queue_size = 7;
    
    ret = spi_bus_add_device(BME_SPI_HOST, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    // --- BSEC Library Initialization ---
    // Initialize the library using the generic interface with function pointers
    if (!envSensor.begin(BME68X_SPI_INTF, bme_spi_read, bme_spi_write, bme_delay_us, &spi_handle, bme_millis)) {
        ESP_LOGE(TAG, "Failed to initialize BSEC library.");
        checkBsecStatus(envSensor);
    }


    // Set temperature offset based on sample rate 
	if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP)
	{
		envSensor.setTemperatureOffset(TEMP_OFFSET_ULP);
	}
	else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP)
	{
		envSensor.setTemperatureOffset(TEMP_OFFSET_LP);
	}

    // Define the list of sensors to subscribe to
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_GAS_PERCENTAGE,
        BSEC_OUTPUT_COMPENSATED_GAS
    };

    // Update the subscription
    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE)) {
        ESP_LOGE(TAG, "Failed to update subscription.");
        checkBsecStatus(envSensor);
    }

    // Attach the callback function for new data
    envSensor.attachCallback(newDataCallback);

    ESP_LOGI(TAG, "BSEC library version %d.%d.%d.%d",
             envSensor.version.major,
             envSensor.version.minor,
             envSensor.version.major_bugfix,
             envSensor.version.minor_bugfix); 

    // --- Main Loop ---
    while(1) {
        if (!envSensor.run()) {
            checkBsecStatus(envSensor);
        }
        // Small delay to allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Blinks the panic LED indefinitely in case of a fatal error.
 */
void errLeds(void)
{
    while(1)
    {
        gpio_set_level(PANIC_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(ERROR_DUR));
        gpio_set_level(PANIC_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(ERROR_DUR));
    }
}

/**
 * @brief Callback function for when new sensor data is available.
 */
void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs) {
        return;
    }

    ESP_LOGI(TAG, "BSEC outputs:");
    ESP_LOGI(TAG, "\tTime stamp = %lld", outputs.output[0].time_stamp / INT64_C(1000000));

    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id) {
            case BSEC_OUTPUT_IAQ:
                ESP_LOGI(TAG, "\tIAQ = %.2f", output.signal);
                ESP_LOGI(TAG, "\tIAQ accuracy = %d", (int)output.accuracy);
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                ESP_LOGI(TAG, "\tTemperature = %.2f C", output.signal);
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                ESP_LOGI(TAG, "\tPressure = %.2f hPa", output.signal);
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                ESP_LOGI(TAG, "\tHumidity = %.2f %%", output.signal);
                break;
            case BSEC_OUTPUT_RAW_GAS:
                ESP_LOGI(TAG, "\tGas resistance = %.0f Ohms", output.signal);
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
                ESP_LOGI(TAG, "\tStabilization status = %d", (int)output.signal);
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
                ESP_LOGI(TAG, "\tRun in status = %d", (int)output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                ESP_LOGI(TAG, "\tCompensated temperature = %.2f C", output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                ESP_LOGI(TAG, "\tCompensated humidity = %.2f %%", output.signal);
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
                ESP_LOGI(TAG, "\tStatic IAQ = %.2f", output.signal);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                ESP_LOGI(TAG, "\tCO2 Equivalent = %.2f ppm", output.signal);
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                ESP_LOGI(TAG, "\tbVOC equivalent = %.2f ppm", output.signal);
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE:
                ESP_LOGI(TAG, "\tGas percentage = %.2f", output.signal);
                break;
            case BSEC_OUTPUT_COMPENSATED_GAS:
                 ESP_LOGI(TAG, "\tCompensated gas = %.0f Ohms", output.signal);
                 break;
            default:
                break;
        }
    }
}

/**
 * @brief Checks the BSEC and BME68x status codes and logs them. Halts on error.
 */
void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK) {
        ESP_LOGE(TAG, "BSEC error code: %d", (int)bsec.status);
        errLeds(); // Halt in case of failure
    } else if (bsec.status > BSEC_OK) {
        ESP_LOGW(TAG, "BSEC warning code: %d", (int)bsec.status);
    }

    if (bsec.sensor.checkStatus() < BME68X_OK) {
        ESP_LOGE(TAG, "BME68X error code: %d", (int)bsec.sensor.checkStatus());
        errLeds(); // Halt in case of failure
    } else if (bsec.sensor.checkStatus() > BME68X_OK) {
        ESP_LOGW(TAG, "BME68X warning code: %d", (int)bsec.sensor.checkStatus());
    }
}
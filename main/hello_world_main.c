/*
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this software is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * This code demonstrates how to interface with a Bosch BME688 sensor over the SPI bus
 * on an ESP32-S3 using the ESP-IDF framework and the Bosch BME68x-Library.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "bme68x.h" // This is the header for the Bosch library you added
#include "rom/ets_sys.h" // For ets_delay_us

// Tag for logging messages to the console
static const char *TAG = "BME688_SPI";

// --- Pin Definitions for SPI Bus 2 on ESP32-S3 DevKitC-1 ---
// These GPIOs are the standard pins for SPI2 on the S3 board.
#define BME688_SPI_HOST SPI2_HOST // The SPI bus we are using (SPI2_HOST is for pins 11,12,13)
#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK GPIO_NUM_12
#define PIN_NUM_CS GPIO_NUM_10 // Chip Select pin to enable communication with the BME688

// Global handle for the SPI device
static spi_device_handle_t spi_handle;

// Struct to hold sensor data
static struct bme68x_dev bme68x_sensor;

// --- Function Prototypes for communication callbacks ---
// The BME68x library requires these functions to be implemented by the user.
// These functions use the ESP-IDF SPI driver to perform the actual data transfer.
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

// --- Chip Select Control ---
// The BME68x library expects a simple way to enable/disable the Chip Select line.
void user_cs_on() {
    gpio_set_level(PIN_NUM_CS, 0); // Active low
}
void user_cs_off() {
    gpio_set_level(PIN_NUM_CS, 1); // De-assert
}

// --- SPI Read Function ---
// This function reads a specified number of bytes from the BME688.
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t handle = *(spi_device_handle_t *)intf_ptr;
    esp_err_t ret;
    uint8_t tx_buf[1] = { reg_addr | 0x80 }; // MSB=1 for read
    uint8_t rx_buf[len + 1];

    spi_transaction_t t = {
        .length = (len + 1) * 8,   // total bits to send/receive
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };

    gpio_set_level(PIN_NUM_CS, 0);
    ret = spi_device_transmit(handle, &t);
    gpio_set_level(PIN_NUM_CS, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    memcpy(reg_data, &rx_buf[1], len); // skip the address byte
    return BME68X_OK;
}

// --- SPI Write Function ---
// This function writes a specified number of bytes to the BME688.
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t handle = *(spi_device_handle_t *)intf_ptr;
    esp_err_t ret;
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg_addr & 0x7F; // MSB=0 for write
    memcpy(&tx_buf[1], reg_data, len);

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf
    };

    gpio_set_level(PIN_NUM_CS, 0);
    ret = spi_device_transmit(handle, &t);
    gpio_set_level(PIN_NUM_CS, 1);

    return (ret == ESP_OK) ? BME68X_OK : ret;
}


// --- Delay Function ---
// The BME68x library needs a microsecond delay function.
void user_delay_us(uint32_t period, void *intf_ptr) {
    ets_delay_us(period);
}


// --- Main Application ---
void app_main(void) {
    ESP_LOGI(TAG, "Initializing SPI bus...");
    
    // Step 1: Configure the SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Step 2: Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(BME688_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Step 3: Configure the device (BME688)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 0, // SPI mode 0
        .spics_io_num = -1, // We control the CS pin manually, so set to -1
        .queue_size = 7, // We can queue 7 transactions
    };

    // Step 4: Add the device to the SPI bus
    ret = spi_bus_add_device(BME688_SPI_HOST, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI bus initialized and device added.");

    // Step 5: Configure the CS pin as a GPIO for manual control
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Set CS pin high to de-select the device initially
    gpio_set_level(PIN_NUM_CS, 1);

    // Step 6: Initialize the BME68x sensor struct with our callback functions
    bme68x_sensor.read = user_spi_read;
    bme68x_sensor.write = user_spi_write;
    bme68x_sensor.delay_us = user_delay_us;
    bme68x_sensor.intf = BME68X_SPI_INTF; // Using SPI interface
    bme68x_sensor.intf_ptr = &spi_handle; // Pass the SPI handle to the callbacks

    // Step 7: Perform the sensor initialization and check for errors
    int8_t rslt = bme68x_init(&bme68x_sensor);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME688 initialization failed with error code: %d", rslt);
        return;
    }

    ESP_LOGI(TAG, "BME688 initialized successfully.");

    // Step 8: Configure the sensor's oversampling and filter settings
    uint8_t op_mode = BME68X_FORCED_MODE;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;

    // Use default values
    bme68x_get_conf(&conf, &bme68x_sensor);
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    bme68x_set_conf(&conf, &bme68x_sensor);

    // Configure heater settings for gas measurement
    bme68x_get_heatr_conf(&heatr_conf, &bme68x_sensor);
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 320; // 320 C
    heatr_conf.heatr_dur = 150;  // 150 ms
    bme68x_set_heatr_conf(op_mode, &heatr_conf, &bme68x_sensor);

    // Step 9: Main loop to continuously read and print data
    while (1) {
        // We set the operating mode to BME68X_FORCED_MODE to trigger a single measurement
        rslt = bme68x_set_op_mode(op_mode, &bme68x_sensor);
        if (rslt != BME68X_OK) {
            ESP_LOGE(TAG, "Failed to set op mode");
        }

        // Wait for the measurement to complete
        uint32_t delay_period = bme68x_get_meas_dur(op_mode, &conf, &bme68x_sensor);
        vTaskDelay(pdMS_TO_TICKS(delay_period / 1000 + 1));

        // Read the sensor data
        struct bme68x_data data;
        uint8_t n_fields;
        rslt = bme68x_get_data(op_mode, &data, &n_fields, &bme68x_sensor);
        ESP_LOGI(TAG, "bme68x_get_data result: %d, n_fields: %d", rslt, n_fields);
        if (rslt == BME68X_OK && n_fields > 0) {
            ESP_LOGI(TAG, "Temperature: %.2f °C", data.temperature);
            ESP_LOGI(TAG, "Humidity: %.2f %%", data.humidity);
            ESP_LOGI(TAG, "Pressure: %.2f hPa", data.pressure / 100.0);
            ESP_LOGI(TAG, "Gas resistance: %lu Ohms", (long unsigned int)data.gas_resistance);
        } else {
            ESP_LOGE(TAG, "Failed to read data from BME688. rslt=%d, n_fields=%d", rslt, n_fields);
        }
        
        // Wait for 2 seconds before the next measurement
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

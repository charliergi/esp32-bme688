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

#include "rom/ets_sys.h" // For ets_delay_us

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mqtt_client.h"
// #include "bsec2_control/bsec2_control.h"
#include "bme688_control/bme688_control.h"
#include "mqtt/mqtt.h"  // Include mqtt.h which provides access to mqtt_client


// Global handle for the SPI device
static spi_device_handle_t spi_handle;

// Struct to hold sensor data
static struct bme68x_dev bme68x_sensor;

// MQTT client is now imported from mqtt.cpp via the mqtt.h header

// --- Main Application ---
extern "C" void app_main(void) {
    // Load environment variables
    load_env_vars();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi
    wifi_init_sta();

    // Initialize MQTT
    mqtt_app_start();
    static const char *TAG = "BME688_SPI";
    ESP_LOGI(TAG, "Initializing SPI bus...");
    
    // Step 1: Configure the SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.max_transfer_sz = 4000;

    // Step 2: Initialize the SPI bus
    ret = spi_bus_initialize(BME688_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Step 3: Configure the device (BME688)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 7;

    // Step 4: Add the device to the SPI bus
    ret = spi_bus_add_device(BME688_SPI_HOST, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI bus initialized and device added.");

    // Step 5: Configure the CS pin as a GPIO for manual control
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Use enum value
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Use enum value
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
            char topic[64];
            char payload[64];

            if (data.status & BME68X_NEW_DATA_MSK) {
                ESP_LOGI(TAG, "Temperature: %.2f °C", data.temperature);
                ESP_LOGI(TAG, "Humidity: %.2f %%", data.humidity);
                ESP_LOGI(TAG, "Pressure: %.2f hPa", data.pressure / 100.0);

                snprintf(topic, sizeof(topic), "esp32/bme688/temperature");
                snprintf(payload, sizeof(payload), "%.2f", data.temperature);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);

                snprintf(topic, sizeof(topic), "esp32/bme688/humidity");
                snprintf(payload, sizeof(payload), "%.2f", data.humidity);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);

                snprintf(topic, sizeof(topic), "esp32/bme688/pressure");
                snprintf(payload, sizeof(payload), "%.2f", data.pressure / 100.0);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
            }

            if (data.status & BME68X_GASM_VALID_MSK) {
                ESP_LOGI(TAG, "Gas resistance: %lu Ohms", (long unsigned int)data.gas_resistance);
                snprintf(topic, sizeof(topic), "esp32/bme688/gas_resistance");
                snprintf(payload, sizeof(payload), "%lu", (long unsigned int)data.gas_resistance);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
            }

        } else {
            ESP_LOGE(TAG, "Failed to read data from BME688. rslt=%d, n_fields=%d", rslt, n_fields);
        }
        
        // Wait for 2 seconds before the next measurement
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

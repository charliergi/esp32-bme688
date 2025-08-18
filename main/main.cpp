/*
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this software is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * This code demonstrates how to interface with a Bosch BME688 sensor using the BSEC2 library
 * on an ESP32-S3 using the ESP-IDF framework to predict CO2 levels and publish via MQTT.
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
#include "bsec2_control/bsec2_control.h"
#include "bme688_control/bme688_control.h"
#include "mqtt/mqtt.h"  // Include mqtt.h which provides access to mqtt_client


// Global handle for the SPI device
static spi_device_handle_t spi_handle;

// Struct to hold sensor data
static struct bme68x_dev bme68x_sensor;

// Task handle for BSEC2 task
TaskHandle_t bsec2_task_handle = NULL;

// BSEC2 task function
void bsec2_task(void *pvParameters) {
    // Run the BSEC2 loop
    bsec2_loop();
    
    // Should never reach here
    vTaskDelete(NULL);
}

// --- Main Application ---
extern "C" void app_main(void) {
    static const char *TAG = "BME688_MAIN";
    
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
    
    ESP_LOGI(TAG, "Initializing BSEC2 for BME688 sensor...");
    
    // Initialize BSEC2
    setup();
    
    // Create a task for BSEC2 processing
    xTaskCreate(
        bsec2_task,          // Task function
        "bsec2_task",        // Task name
        8192,                // Stack size (bytes)
        NULL,                // Task parameters
        5,                   // Task priority
        &bsec2_task_handle   // Task handle
    );
    
    ESP_LOGI(TAG, "BSEC2 task created, monitoring CO2 equivalent and other data...");
}

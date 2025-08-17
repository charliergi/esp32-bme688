#include "bsec2_control.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char *TAG = "BSEC2";

// I2C configuration
#define I2C_MASTER_SDA_IO 21  // GPIO for SDA
#define I2C_MASTER_SCL_IO 22  // GPIO for SCL
#define I2C_MASTER_FREQ_HZ 100000  // I2C frequency
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number

// BME688 I2C address
#define BME68X_I2C_ADDR 0x76  // I2C address for BME688

// Forward declarations for I2C communication with BME688
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void delay_us(uint32_t period, void *intf_ptr);
unsigned long get_timestamp_us();

// Bosch BSEC2 Library
Bsec2 envSensor;

// BSEC 2 setup
void setup() {
    ESP_LOGI(TAG, "Initializing BSEC2 and BME688 sensor");
    
    // Initialize I2C for ESP-IDF
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    
    // Initialize BSEC2 with custom I2C functions
    bool result = envSensor.begin(BME68X_I2C_INTF, i2c_read, i2c_write, delay_us, nullptr, get_timestamp_us);
    
    if (!result) {
        ESP_LOGE(TAG, "BSEC2 initialization failed");
        return;
    }
    
    ESP_LOGI(TAG, "BSEC2 initialized successfully");
    
    // Subscribe to IAQ and other outputs
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS
    };
    envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP);
}

// I2C read function for BME688
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0;
    uint8_t dev_id = BME68X_I2C_ADDR; // Use fixed BME688 I2C address
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &reg_data[len-1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        rslt = -1;
    }
    
    return rslt;
}

// I2C write function for BME688
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0;
    uint8_t dev_id = BME68X_I2C_ADDR; // Use fixed BME688 I2C address
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)reg_data, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        rslt = -1;
    }
    
    return rslt;
}

// Delay function for BME688
void delay_us(uint32_t period, void *intf_ptr) {
    ets_delay_us(period);
}

// Get timestamp in milliseconds for BSEC2
unsigned long get_timestamp_us() {
    return (unsigned long)(esp_timer_get_time() / 1000); // convert microseconds to milliseconds
}

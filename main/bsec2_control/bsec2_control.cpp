#include "bsec2_control.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"  // Include for esp_rom_delay_us
#include "../mqtt/mqtt.h"

static const char *TAG = "BSEC2";

// External MQTT client
extern esp_mqtt_client_handle_t mqtt_client;

// I2C configuration
#define I2C_MASTER_SDA_IO 47  // GPIO for SDA (updated pin)
#define I2C_MASTER_SCL_IO 48  // GPIO for SCL (updated pin)
#define I2C_MASTER_FREQ_HZ 100000  // I2C frequency restored to standard 100kHz
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number

// BME688 I2C address
#define BME68X_I2C_ADDR 0x76  // I2C address for BME688 (DFRobot SEN0629 default)
#define BME68X_I2C_ADDR_ALT 0x77  // Alternative I2C address for BME688

// Forward declarations for I2C communication with BME688
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void delay_us(uint32_t period, void *intf_ptr);
unsigned long get_timestamp_us();

// I2C scanner function to detect connected devices
bool scan_i2c_bus() {
    ESP_LOGI(TAG, "-------------------------------------");
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    ESP_LOGI(TAG, "Using SDA pin: GPIO %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "Using SCL pin: GPIO %d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "I2C Frequency: %d Hz", I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "-------------------------------------");
    
    int devices_found = 0;
    bool bme688_found = false;
    
    for (uint8_t i = 1; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", i);
            devices_found++;
            
            // Check if this is our BME688 sensor
            if (i == BME68X_I2C_ADDR || i == BME68X_I2C_ADDR_ALT) {
                ESP_LOGI(TAG, "  --> This appears to be the BME688 sensor!");
                bme688_found = true;
            }
        }
    }
    
    ESP_LOGI(TAG, "-------------------------------------");
    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", devices_found);
    
    if (!bme688_found) {
        ESP_LOGE(TAG, "DFRobot SEN0629 BME688 sensor NOT FOUND! Check your connections:");
        ESP_LOGE(TAG, "  - SDA connected to GPIO %d", I2C_MASTER_SDA_IO);
        ESP_LOGE(TAG, "  - SCL connected to GPIO %d", I2C_MASTER_SCL_IO);
        ESP_LOGE(TAG, "  - Verify power (3.3V) and ground connections");
        ESP_LOGE(TAG, "  - Expected BME688 at address 0x76 or 0x77");
        ESP_LOGE(TAG, "-------------------------------------");
        ESP_LOGE(TAG, "Troubleshooting tips for DFRobot SEN0629:");
        ESP_LOGE(TAG, "1. Check for loose wires or bad connections");
        ESP_LOGE(TAG, "2. Ensure the module is properly powered (3.3V)");
        ESP_LOGE(TAG, "3. Make sure the module is in I2C mode, not SPI mode");
        ESP_LOGE(TAG, "4. Try shorter wires if using long connections");
        ESP_LOGE(TAG, "5. Verify the sensor is not damaged");
        ESP_LOGE(TAG, "-------------------------------------");
    }
    
    return bme688_found;
}

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
    
    // Configure and install I2C driver
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C configuration failed");
        return;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return;
    }
    
    // Add a small delay to ensure I2C is ready
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Scan I2C bus to verify BME688 connection
    if (!scan_i2c_bus()) {
        ESP_LOGE(TAG, "BME688 sensor not detected on I2C bus - check wiring!");
        // We'll continue anyway, but sensor operations will likely fail
    }
    
    // Initialize BSEC2 with custom I2C functions
    bool result = envSensor.begin(BME68X_I2C_INTF, i2c_read, i2c_write, delay_us, nullptr, get_timestamp_us);
    
    if (!result) {
        ESP_LOGE(TAG, "BSEC2 initialization failed");
        return;
    }
    
    // Check BSEC status after initialization
    checkBsecStatus(envSensor);
    
    ESP_LOGI(TAG, "BSEC2 initialized successfully");
    
    // Set callback for new data
    envSensor.attachCallback(newDataCallback);
    
    // Subscribe to IAQ, CO2, and other outputs
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS
    };
    
    // Use ULP (Ultra Low Power) instead of LP (Low Power) for more frequent sampling
    int status = envSensor.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_ULP);
    if (status != BSEC_OK) {
        ESP_LOGW(TAG, "BSEC subscription update failed with status: %d", status);
    } else {
        ESP_LOGI(TAG, "BSEC subscription updated successfully");
    }
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
    // Use esp_rom_delay_us for microsecond delay
    esp_rom_delay_us(period);
}

// Get timestamp in microseconds for BSEC2
unsigned long get_timestamp_us() {
    return (unsigned long)esp_timer_get_time();
}

// Process sensor readings and publish via MQTT
void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec) {
    char topic[64];
    char payload[64];
    
    // Log and publish the number of outputs
    ESP_LOGI(TAG, "Number of BSEC outputs: %d", outputs.nOutputs);
    
    // Iterate through all the outputs
    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData& output = outputs.output[i];
        
        switch (output.sensor_id) {
            case BSEC_OUTPUT_IAQ:
                ESP_LOGI(TAG, "IAQ: %.2f (Accuracy: %d)", output.signal, output.accuracy);
                snprintf(topic, sizeof(topic), "esp32/bme688/iaq");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                
                // Also publish accuracy
                snprintf(topic, sizeof(topic), "esp32/bme688/iaq_accuracy");
                snprintf(payload, sizeof(payload), "%d", output.accuracy);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                ESP_LOGI(TAG, "CO2 equivalent: %.2f ppm (Accuracy: %d)", output.signal, output.accuracy);
                snprintf(topic, sizeof(topic), "esp32/bme688/co2_equivalent");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                
                // Also publish accuracy
                snprintf(topic, sizeof(topic), "esp32/bme688/co2_accuracy");
                snprintf(payload, sizeof(payload), "%d", output.accuracy);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                ESP_LOGI(TAG, "Breath VOC: %.2f ppm (Accuracy: %d)", output.signal, output.accuracy);
                snprintf(topic, sizeof(topic), "esp32/bme688/breath_voc");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                ESP_LOGI(TAG, "Temperature: %.2f °C", output.signal);
                snprintf(topic, sizeof(topic), "esp32/bme688/temperature");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_RAW_PRESSURE:
                ESP_LOGI(TAG, "Pressure: %.2f hPa", output.signal / 100.0);
                snprintf(topic, sizeof(topic), "esp32/bme688/pressure");
                snprintf(payload, sizeof(payload), "%.2f", output.signal / 100.0);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_RAW_HUMIDITY:
                ESP_LOGI(TAG, "Humidity: %.2f %%", output.signal);
                snprintf(topic, sizeof(topic), "esp32/bme688/humidity");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            case BSEC_OUTPUT_RAW_GAS:
                ESP_LOGI(TAG, "Gas resistance: %.2f Ohms", output.signal);
                snprintf(topic, sizeof(topic), "esp32/bme688/gas_resistance");
                snprintf(payload, sizeof(payload), "%.2f", output.signal);
                esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
                break;
                
            default:
                ESP_LOGI(TAG, "Unknown output sensor id: %d, signal: %.2f", output.sensor_id, output.signal);
                break;
        }
    }
}

// Check BSEC status and log errors
void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
        ESP_LOGE(TAG, "BSEC error code: %d", bsec.status);
    } else if (bsec.status > BSEC_OK) {
        ESP_LOGW(TAG, "BSEC warning code: %d", bsec.status);
    }
    
    // Skip BME68x status checking for now until we determine the correct member name
}

// Main function to run the BSEC loop
void bsec2_loop() {
    ESP_LOGI(TAG, "Starting BSEC2 loop");
    
    // Initial warm-up cycle counter
    int warmup_cycles = 0;
    const int REQUIRED_WARMUP_CYCLES = 5;
    
    // Run BSEC loop to continuously collect and process data
    while (1) {
        // Process data if available
        if (envSensor.run()) {
            // Data processing happens in the callback
            ESP_LOGI(TAG, "BSEC run successful");
            warmup_cycles++; // Increment warm-up counter on success
        } else {
            ESP_LOGW(TAG, "BSEC run failed");
            
            // Check BSEC status
            checkBsecStatus(envSensor);
            
            // If we get warning code 100 (BSEC_W_INSUFFICIENT_DATA) during warm-up, it's normal
            if (envSensor.status == 100 && warmup_cycles < REQUIRED_WARMUP_CYCLES) {
                ESP_LOGI(TAG, "BSEC needs more data for calibration (warm-up cycle %d/%d)", 
                         warmup_cycles, REQUIRED_WARMUP_CYCLES);
            }
        }
        
        // Wait for next measurement cycle
        // Using a shorter interval for more frequent sampling during warm-up
        int delay_ms = (warmup_cycles < REQUIRED_WARMUP_CYCLES) ? 1000 : 3000;
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

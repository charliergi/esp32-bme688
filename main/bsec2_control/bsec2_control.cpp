extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h" // For ets_delay_us
}

#include "bsec2.h"
#include "bsec2_control.h"


// // Forward declarations
// void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec);
// void checkBsecStatus(Bsec2 bsec);
// void errLeds();

// Platform-specific functions for BSEC2 library
int8_t bme_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t handle = *(spi_device_handle_t *)intf_ptr;
    esp_err_t ret;

    ESP_LOGD(TAG, "SPI Write: reg=0x%02X, len=%lu", reg_addr, len);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
// ...existing code...
    t.length = 8; // 8 bits for the register address

    // Transmit the register address
    ret = spi_device_polling_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Write reg_addr failed: %s", esp_err_to_name(ret));
        return -1;
    }

    // Transmit the data
    if (len > 0) {
        memset(&t, 0, sizeof(t));
        t.tx_buffer = reg_data;
        t.length = len * 8; // length is in bits
        ret = spi_device_polling_transmit(handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI Write data failed: %s", esp_err_to_name(ret));
            return -1;
        }
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

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "SPI Read: reg=0x%02X, len=%lu", reg_addr, len);
        if (len > 0) {
            char data_str[len * 3 + 1];
            for (int i = 0; i < len; i++) {
                sprintf(data_str + i * 3, "%02X ", reg_data[i]);
            }
            ESP_LOGD(TAG, "  Data: %s", data_str);
        }
        return 0;
    } else {
        ESP_LOGE(TAG, "SPI Read failed: %s", esp_err_to_name(ret));
        return -1;
    }
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    ets_delay_us(period);
}

unsigned long get_millis() {
    return (unsigned long) (esp_timer_get_time() / 1000);
}

// extern "C" void app_main(void)
// {
//     // Init LED
//     gpio_set_direction(PANIC_LED, GPIO_MODE_OUTPUT);

//     // Init SPI bus
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = GPIO_NUM_11,
//         .miso_io_num = GPIO_NUM_13,
//         .sclk_io_num = GPIO_NUM_12,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .data4_io_num = -1,
//         .data5_io_num = -1,
//         .data6_io_num = -1,
//         .data7_io_num = -1,
//         .data_io_default_level = 0,
//         .max_transfer_sz = 0,
//         .flags = 0,
//         .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
//         .intr_flags = 0,
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

//     spi_device_interface_config_t devcfg = {};  // Zero-initialize all fields
//     devcfg.clock_speed_hz = 1 * 1000 * 1000;
//     devcfg.spics_io_num = PIN_CS;
//     devcfg.queue_size = 1;
//     ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

//     // Init BSEC2
//     if (!envSensor.begin(BME68X_SPI_INTF, bme68x_spi_read, bme68x_spi_write, bme68x_delay_us, &spi, get_millis)) {
//         checkBsecStatus(envSensor);
//     }

//     if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP) {
//         envSensor.setTemperatureOffset(2.0f);
//     } else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP) {
//         envSensor.setTemperatureOffset(1.0f);
//     }

//     bsecSensor sensorList[] = {
//         BSEC_OUTPUT_IAQ,
//         BSEC_OUTPUT_RAW_TEMPERATURE,
//         BSEC_OUTPUT_RAW_PRESSURE,
//         BSEC_OUTPUT_RAW_HUMIDITY,
//         BSEC_OUTPUT_RAW_GAS,
//         BSEC_OUTPUT_STATIC_IAQ,
//         BSEC_OUTPUT_CO2_EQUIVALENT,
//         BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
//     };

//     if (!envSensor.updateSubscription(sensorList, sizeof(sensorList)/sizeof(sensorList[0]), SAMPLE_RATE)) {
//         checkBsecStatus(envSensor);
//     }

//     envSensor.attachCallback(newDataCallback);

//     ESP_LOGI(TAG, "BSEC library version %d.%d.%d.%d",
//              envSensor.version.major, envSensor.version.minor,
//              envSensor.version.major_bugfix, envSensor.version.minor_bugfix);

//     // Replace Arduino loop()
//     while (true) {
//         if (!envSensor.run()) {
//             checkBsecStatus(envSensor);
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
// void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec)
// {
//     if (!outputs.nOutputs) return;

//     ESP_LOGI(TAG, "Timestamp %lld ms", outputs.output[0].time_stamp / 1000000);

//     for (uint8_t i = 0; i < outputs.nOutputs; i++) {
//         const bsecData output = outputs.output[i];
//         switch (output.sensor_id) {
//             case BSEC_OUTPUT_IAQ:
//                 ESP_LOGI(TAG, "IAQ %.2f (acc %d)", output.signal, output.accuracy);
//                 break;
//             case BSEC_OUTPUT_RAW_TEMPERATURE:
//                 ESP_LOGI(TAG, "Temperature %.2f °C", output.signal);
//                 break;
//             case BSEC_OUTPUT_RAW_PRESSURE:
//                 ESP_LOGI(TAG, "Pressure %.2f hPa", output.signal);
//                 break;
//             case BSEC_OUTPUT_RAW_HUMIDITY:
//                 ESP_LOGI(TAG, "Humidity %.2f %%", output.signal);
//                 break;
//             case BSEC_OUTPUT_RAW_GAS:
//                 ESP_LOGI(TAG, "Gas resistance %.2f", output.signal);
//                 break;
//             case BSEC_OUTPUT_CO2_EQUIVALENT:
//                 ESP_LOGI(TAG, "CO2eq %.2f ppm", output.signal);
//                 break;
//             case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
//                 ESP_LOGI(TAG, "bVOCeq %.2f ppm", output.signal);
//                 break;
//             default:
//                 break;
//         }
//     }
// }

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK) {
        ESP_LOGE(TAG, "BSEC error %d", bsec.status);
        errLeds();
    } else if (bsec.status > BSEC_OK) {
        ESP_LOGW(TAG, "BSEC warning %d", bsec.status);
    }
    if (bsec.sensor.checkStatus() < BME68X_OK) {
        ESP_LOGE(TAG, "BME68X error %d", bsec.sensor.checkStatus());
        errLeds();
    } else if (bsec.sensor.checkStatus() > BME68X_OK) {
        ESP_LOGW(TAG, "BME68X warning %d", bsec.sensor.checkStatus());
    }
}
void errLeds() {
    // Assuming PANIC_LED is defined in your header file
    // If not, you'll need to define it or adjust this code
    #ifdef PANIC_LED
    for (int i = 0; i < 3; i++) {
        gpio_set_level(PANIC_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PANIC_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    #else
    ESP_LOGE(TAG, "Error LED notification (LED pin not defined)");
    #endif
}

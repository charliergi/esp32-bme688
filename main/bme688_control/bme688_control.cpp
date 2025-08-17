#include "bme68x.h" // This is the header for the Bosch library you added
#include <stdio.h>
#include <string.h> // For memset
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "rom/ets_sys.h" // For ets_delay_us
#include "bme688_control.h"
#include "rom/ets_sys.h" // For ets_delay_us

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
    // Use uint8_t(value) for explicit cast to avoid narrowing conversion warnings
    uint8_t tx_buf[1] = { uint8_t(reg_addr | 0x80) }; // MSB=1 for read
    uint8_t rx_buf[len + 1];

    spi_transaction_t t;
    // Initialize all fields to zero first
    memset(&t, 0, sizeof(spi_transaction_t));
    // Then set the fields we need
    t.length = (len + 1) * 8;   // total bits to send/receive
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

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

    spi_transaction_t t;
    // Initialize all fields to zero first
    memset(&t, 0, sizeof(spi_transaction_t));
    // Then set the fields we need
    t.length = (len + 1) * 8;
    t.tx_buffer = tx_buf;

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


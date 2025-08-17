#include "bme68x.h"

// --- Pin Definitions for SPI Bus 2 on ESP32-S3 DevKitC-1 ---
// These GPIOs are the standard pins for SPI2 on the S3 board.
#define BME688_SPI_HOST SPI2_HOST // The SPI bus we are using (SPI2_HOST is for pins 11,12,13)
#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK GPIO_NUM_12
#define PIN_NUM_CS GPIO_NUM_10 // Chip Select pin to enable communication with the BME688
// Tag for logging messages to the console

int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);
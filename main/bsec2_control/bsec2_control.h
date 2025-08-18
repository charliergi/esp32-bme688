
// Bosch BSEC2 Library
#include <bsec2.h>
#include "mqtt_client.h"

// External reference to MQTT client
extern esp_mqtt_client_handle_t mqtt_client;

/**
 * @brief : Initialize BSEC2 sensor with I2C
 */
void setup();

/**
 * @brief : Main loop for BSEC2 operation
 */
void bsec2_loop();

/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68x_data data, const bsecOutputs outputs, Bsec2 bsec);

/**
 * @brief : Scan I2C bus for devices and print their addresses
 * @return : true if BME688 sensor found (at address 0x76 or 0x77), false otherwise
 */
bool scan_i2c_bus();

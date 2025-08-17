idf_component_register(SRCS "main.cpp"
                       "bme688_control/bme688_control.cpp"
                       "bsec2_control/bsec2_control.cpp"
                       "mqtt/mqtt.cpp"
                       PRIV_REQUIRES spi_flash driver Bosch-BME68x-Library Bosch-BSEC2-Library nvs_flash esp_netif esp_event esp_wifi mqtt
                       INCLUDE_DIRS "." "bme688_control" "bsec2_control" "mqtt"
                       EMBED_TXTFILES ".env")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -u _printf_float")
target_link_libraries(${COMPONENT_LIB} PRIVATE m)

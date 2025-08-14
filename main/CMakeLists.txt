idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash driver Bosch-BME68x-Library nvs_flash esp_netif esp_event esp_wifi mqtt
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ".env")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -u _printf_float")
target_link_libraries(${COMPONENT_LIB} PRIVATE m)

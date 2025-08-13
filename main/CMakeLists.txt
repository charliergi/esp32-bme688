idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash driver Bosch-BME68x-Library
                       INCLUDE_DIRS ".")

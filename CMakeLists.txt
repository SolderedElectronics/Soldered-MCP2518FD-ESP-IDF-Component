idf_component_register(SRCS "src/soldered_mcp2518fd.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_spi)

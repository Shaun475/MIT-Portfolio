idf_component_register(
    SRCS "hello_world_main.c"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES spi_flash esp_driver_ledc
    INCLUDE_DIRS ""
)
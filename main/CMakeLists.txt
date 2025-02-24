idf_component_register(SRCS "i2c_demo.c" "lcd_display.c"
    PRIV_REQUIRES esp_driver_i2c
    INCLUDE_DIRS ".")
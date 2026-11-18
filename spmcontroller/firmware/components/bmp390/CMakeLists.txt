idf_component_register(
    SRCS "driver_bmp390.c"   
         "driver_bmp390_interface.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c
)
idf_component_register(
    SRCS "lsm6dsv80x_reg.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c
)
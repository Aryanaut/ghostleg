idf_component_register(
    SRCS "mmc5983ma.c" "matrix_utils.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c
)

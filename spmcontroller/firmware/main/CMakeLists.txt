idf_component_register(SRCS "sim_replay.c" "main.c" "bridge.c" "tb_log.c" "imu_polling.c" "imu_link.c"
                            "nor_log.c" "storage.c" "sdcard.c"
                            "sensor_data.c" "barometer.c" "magnetometer.c" "usb_stream.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_uart esp_driver_i2c esp_driver_usb_serial_jtag esp_ringbuf esp_timer
                                     spi_flash nvs_flash fatfs bmp390 mmc5983ma lsm6dsv80x
                       INCLUDE_DIRS ""
                       )

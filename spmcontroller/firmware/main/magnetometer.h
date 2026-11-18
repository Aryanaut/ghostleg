#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <driver/i2c_master.h>
#include <esp_err.h>

esp_err_t magnetometer_init(i2c_master_bus_handle_t _i2c_bus_hdl);
void magnetometer_update(void);

#endif // MAGNETOMETER_H
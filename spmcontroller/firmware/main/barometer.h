#ifndef BAROMETER_H
#define BAROMETER_H
#include <driver/i2c_master.h>
#include <esp_err.h>

// Chip: BMP390 

esp_err_t baro_init(i2c_master_bus_handle_t _i2c_bus_hdl);
void barometer_update(void);

#endif // BAROMETER_H
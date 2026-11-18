#include "barometer.h"
#include "sim_replay.h"
#include "bridge.h"
#include "sensor_data.h"

#include <esp_log.h>
#include <driver_bmp390_interface.h>
#include <driver_bmp390.h>

#define TAG "baro"

// On this dev board the BMP390's SDO pin is tied to +3.3V, which sets the
// I2C address LSB to 1 -> 0x77.
#define BMP390_I2C_ADDR 0x77

static bmp390_handle_t gs_handle;
static bool initialised = false;

esp_err_t baro_init(i2c_master_bus_handle_t i2c_bus_hdl) {
    uint8_t res;

    bmp390_set_i2c_config(i2c_bus_hdl, BMP390_I2C_ADDR, 400000); // 400 kHz I2C

    /* link interface functions */
    DRIVER_BMP390_LINK_INIT(&gs_handle, bmp390_handle_t);
    DRIVER_BMP390_LINK_IIC_INIT(&gs_handle, bmp390_interface_iic_init);
    DRIVER_BMP390_LINK_IIC_DEINIT(&gs_handle, bmp390_interface_iic_deinit);
    DRIVER_BMP390_LINK_IIC_READ(&gs_handle, bmp390_interface_iic_read);
    DRIVER_BMP390_LINK_IIC_WRITE(&gs_handle, bmp390_interface_iic_write);
    DRIVER_BMP390_LINK_SPI_INIT(&gs_handle, bmp390_interface_spi_init);
    DRIVER_BMP390_LINK_SPI_DEINIT(&gs_handle, bmp390_interface_spi_deinit);
    DRIVER_BMP390_LINK_SPI_READ(&gs_handle, bmp390_interface_spi_read);
    DRIVER_BMP390_LINK_SPI_WRITE(&gs_handle, bmp390_interface_spi_write);
    DRIVER_BMP390_LINK_DELAY_MS(&gs_handle, bmp390_interface_delay_ms);
    DRIVER_BMP390_LINK_DEBUG_PRINT(&gs_handle, bmp390_interface_debug_print);

    res = bmp390_set_interface(&gs_handle, BMP390_INTERFACE_IIC);
    if (res != 0) { ESP_LOGE(TAG, "set interface failed"); return ESP_FAIL; }

    res = bmp390_set_addr_pin(&gs_handle, BMP390_ADDRESS_ADO_HIGH);
    if (res != 0) { ESP_LOGE(TAG, "set addr pin failed"); return ESP_FAIL; }

    res = bmp390_init(&gs_handle);
    if (res != 0) { ESP_LOGE(TAG, "init failed"); return ESP_FAIL; }

    res = bmp390_set_iic_watchdog_timer(&gs_handle, BMP390_BOOL_TRUE);
    if (res != 0) { ESP_LOGE(TAG, "set iic watchdog timer failed"); goto fail; }

    res = bmp390_set_iic_watchdog_period(&gs_handle, BMP390_IIC_WATCHDOG_PERIOD_1P25_MS);
    if (res != 0) { ESP_LOGE(TAG, "set iic watchdog period failed"); goto fail; }

    res = bmp390_set_fifo(&gs_handle, BMP390_BOOL_FALSE);
    if (res != 0) { ESP_LOGE(TAG, "set fifo failed"); goto fail; }

    res = bmp390_set_pressure(&gs_handle, BMP390_BOOL_TRUE);
    if (res != 0) { ESP_LOGE(TAG, "set pressure failed"); goto fail; }

    res = bmp390_set_temperature(&gs_handle, BMP390_BOOL_TRUE);
    if (res != 0) { ESP_LOGE(TAG, "set temperature failed"); goto fail; }

    res = bmp390_set_pressure_oversampling(&gs_handle, BMP390_OVERSAMPLING_x1);
    if (res != 0) { ESP_LOGE(TAG, "set pressure oversampling failed"); goto fail; }

    res = bmp390_set_temperature_oversampling(&gs_handle, BMP390_OVERSAMPLING_x1);
    if (res != 0) { ESP_LOGE(TAG, "set temperature oversampling failed"); goto fail; }

    res = bmp390_set_odr(&gs_handle, BMP390_ODR_200_HZ);
    if (res != 0) { ESP_LOGE(TAG, "set odr failed"); goto fail; }

    res = bmp390_set_filter_coefficient(&gs_handle, BMP390_FILTER_COEFFICIENT_15);
    if (res != 0) { ESP_LOGE(TAG, "set filter coefficient failed"); goto fail; }

    res = bmp390_set_mode(&gs_handle, BMP390_MODE_NORMAL_MODE);
    if (res != 0) { ESP_LOGE(TAG, "set mode failed"); goto fail; }

    ESP_LOGI(TAG, "barometer initialised successfully");
    initialised = true;
    return ESP_OK;

fail:
    (void)bmp390_deinit(&gs_handle);
    return ESP_FAIL;
}

void barometer_update(void) {
    if (!initialised) return;

    uint8_t status;
    if (bmp390_get_status(&gs_handle, &status) != 0) {
        ESP_LOGE(TAG, "get status failed");
        return;
    }
    // Skip if a fresh pressure + temperature sample isn't ready yet.
    if ((status & BMP390_STATUS_PRESS_READY) == 0 ||
        (status & BMP390_STATUS_TEMP_READY) == 0) {
        return;
    }

    float temp = 0, pressure = 0;
    uint32_t temp_raw = 0, pressure_raw = 0;
    if (bmp390_read_temperature_pressure(&gs_handle, &temp_raw, &temp,
                                         &pressure_raw, &pressure) != 0) {
        ESP_LOGE(TAG, "read failed");
        return;
    }
    pressure = pressure / 100.0f; // Pa -> hPa

#if SIM_REPLAY
    // The read above was real, so its timing and its data-ready cadence are
    // real; only the value is replaced. The altitude comes from the bridge's
    // sim clock, so the barometer and the IMU cannot drift apart.
    pressure = sim_replay_pressure_hpa(bridge_sim_baro_alt_m());
#endif

    sensor_data_update_baro(pressure, temp);
}

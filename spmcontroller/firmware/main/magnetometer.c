#include "magnetometer.h"
#include "sensor_data.h"
#include "usb_stream.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <mmc5983ma.h>

#define TAG "mag"

static mmc5983ma_t mag;
static bool initialised = false;
static float last_temp = 0.0f;

esp_err_t magnetometer_init(i2c_master_bus_handle_t i2c_bus_hdl) {
    ESP_LOGI(TAG, "magnetometer init start");
    esp_err_t err = mmc5983ma_init(&mag, i2c_bus_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mmc5983ma init failed");
        return err;
    }
    err = mmc5983ma_set_bandwidth(&mag, MMC5983MA_BW_800HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mmc5983ma set bandwidth failed");
        return err;
    }

    initialised = true;
    ESP_LOGI(TAG, "magnetometer init done");
    return ESP_OK;
}

void magnetometer_update(void) {
    if (!initialised) return;

    static TickType_t last_set_reset = 0;
    static TickType_t last_temp_read = 0;
    float mag_data[3];
    esp_err_t ret;

    TickType_t now = xTaskGetTickCount();

    // A periodic SET/RESET measurement cancels the sensor's offset drift.
    // In between, take ordinary readings.
    if (now - last_set_reset > pdMS_TO_TICKS(500)) {
        ret = mmc5983ma_set_reset_measurement(&mag, &mag_data[0], &mag_data[1], &mag_data[2]);
        last_set_reset = now;
    } else {
        ret = mmc5983ma_read_mag_ut(&mag, &mag_data[0], &mag_data[1], &mag_data[2]);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read failed: %s", esp_err_to_name(ret));
        return;
    }

    // Temperature changes slowly, so read it once a second.
    if (now - last_temp_read > pdMS_TO_TICKS(1000)) {
        if (mmc5983ma_read_temperature(&mag, &last_temp) != ESP_OK) {
            ESP_LOGE(TAG, "temperature read failed");
        }
        last_temp_read = now;
    }

    sensor_data_update_mag(mag_data, last_temp);
    usb_stream_send_mag(mag_data, last_temp);   // SPM rig: raw field to the PC
}

#include "sensor_data.h"

#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define TAG "sensors"

// The one global copy of the latest sensor readings, guarded by a mutex so the
// sensor task and any reader always see a consistent snapshot.
static SensorData data;
static SemaphoreHandle_t data_mutex;

void sensor_data_init(void) {
    memset(&data, 0, sizeof(data));
    data_mutex = xSemaphoreCreateMutex();
}

void sensor_data_update_imu(const float accel_lg[3], const float accel_hg[3],
                            const float gyro[3], const float imu_temp[1]) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    if (accel_lg) {
        data.accel_lg_x = accel_lg[0];
        data.accel_lg_y = accel_lg[1];
        data.accel_lg_z = accel_lg[2];
    }
    if (accel_hg) {
        data.accel_hg_x = accel_hg[0];
        data.accel_hg_y = accel_hg[1];
        data.accel_hg_z = accel_hg[2];
    }
    if (gyro) {
        data.gyro_x = gyro[0];
        data.gyro_y = gyro[1];
        data.gyro_z = gyro[2];
    }
    if (imu_temp) {
        data.imu_temp = imu_temp[0];
    }
    data.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(data_mutex);
}

void sensor_data_update_mag(const float mag[3], float mag_temp) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    if (mag) {
        data.mag_x = mag[0];
        data.mag_y = mag[1];
        data.mag_z = mag[2];
    }
    data.mag_temp = mag_temp;
    data.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(data_mutex);
}

void sensor_data_update_baro(float pressure, float baro_temp) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    data.baro_pressure = pressure;
    data.baro_temp = baro_temp;
    data.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(data_mutex);
}

SensorData sensor_data_get(void) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    SensorData copy = data;
    xSemaphoreGive(data_mutex);
    return copy;
}

void sensor_data_print(void) {
    SensorData d = sensor_data_get();
    ESP_LOGI(TAG,
        "t=%llu us | "
        "accel_lg[mg]=(%.1f, %.1f, %.1f) accel_hg[mg]=(%.1f, %.1f, %.1f) "
        "gyro[mdps]=(%.1f, %.1f, %.1f) imu_temp=%.1f C | "
        "mag[uT]=(%.2f, %.2f, %.2f) mag_temp=%.1f C | "
        "baro=%.2f hPa baro_temp=%.1f C",
        d.timestamp_us,
        d.accel_lg_x, d.accel_lg_y, d.accel_lg_z,
        d.accel_hg_x, d.accel_hg_y, d.accel_hg_z,
        d.gyro_x, d.gyro_y, d.gyro_z, d.imu_temp,
        d.mag_x, d.mag_y, d.mag_z, d.mag_temp,
        d.baro_pressure, d.baro_temp);
}

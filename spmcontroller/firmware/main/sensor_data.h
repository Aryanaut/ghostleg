#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include <stdint.h>

// SensorData holds the most recent reading from every sensor on the board.
// Each sensor driver writes into the one global copy; the sensor task prints it.
typedef struct SensorData {
    // IMU - LSM6DSV80X (SPI2) ------------------------------------------------
    float accel_lg_x, accel_lg_y, accel_lg_z;  // low-g accelerometer   [mg]
    float accel_hg_x, accel_hg_y, accel_hg_z;  // high-g accelerometer  [mg]
    float gyro_x, gyro_y, gyro_z;              // gyroscope             [mdps]
    float imu_temp;                            // IMU temperature       [degC]

    // Magnetometer - MMC5983MA (I2C1) ---------------------------------------
    float mag_x, mag_y, mag_z;                 // magnetic field        [uT]
    float mag_temp;                            // mag temperature       [degC]

    // Barometer - BMP390 (I2C1) ---------------------------------------------
    float baro_pressure;                       // pressure              [hPa]
    float baro_temp;                           // baro temperature      [degC]

    uint64_t timestamp_us;                     // time of last update since boot [us]
} SensorData;

// Create the mutex that guards the global struct. Call once before the sensors.
void sensor_data_init(void);

// Called by each sensor driver after a fresh read. Pass NULL for a group that
// has no new data this cycle (the previous value is kept).
void sensor_data_update_imu(const float accel_lg[3], const float accel_hg[3],
                            const float gyro[3], const float imu_temp[1]);
void sensor_data_update_mag(const float mag[3], float mag_temp);
void sensor_data_update_baro(float pressure, float baro_temp);

// Return a consistent copy of the whole global struct.
SensorData sensor_data_get(void);

// Print the whole global struct to the serial monitor.
void sensor_data_print(void);

#endif // SENSOR_DATA_H

#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>

// Chip: LSM6DSV80X on SPI2 (CS0). Configured at 960 Hz ODR for low-g accel,
// high-g accel and gyro, matching the Avionics C onboard configuration.

#define LG_ACCEL_FULL_SCALE LSM6DSV80X_16g
#define HG_ACCEL_FULL_SCALE LSM6DSV80X_64g
#define GYRO_FULL_SCALE LSM6DSV80X_2000dps

#define LG_ACCEL_TO_MG lsm6dsv80x_from_fs16_to_mg
#define HG_ACCEL_TO_MG lsm6dsv80x_from_fs64_to_mg
#define GYRO_TO_MDPS lsm6dsv80x_from_fs2000_to_mdps

#define IMU_SPI_CLOCK_HZ (10 * 1000 * 1000) // 10 MHz

// One raw sample as read from the sensor. `flags` uses IMU_LINK_FLAG_* bits
// (imu_link.h) to say which groups had new data.
typedef struct {
    int16_t xl[3];
    int16_t hg[3];
    int16_t gy[3];
    int16_t temp;
    uint8_t flags;
} imu_sample_t;

// Largest number of samples one FIFO drain will return. The FIFO is 4 KB, so
// it holds roughly 195 samples (~810 ms at 240 Hz) before it wraps - far more
// than the worst SD card stall measured on the shared SPI bus (79 ms).
#define IMU_FIFO_MAX_SAMPLES  64

// SPI bus must already be initialised. Probes WHO_AM_I, resets and configures
// the sensor, enables the FIFO, and routes data-ready to INT1 (pulsed).
esp_err_t imu_init(void);

// Read status + all output registers in two burst transfers. Returns ESP_OK
// even if nothing was fresh (then sample->flags == 0). Bypasses the FIFO.
esp_err_t imu_read_sample(imu_sample_t *sample);

// Drain the hardware FIFO into `out`, oldest first, at most `max` samples.
// Returns the number written, or -1 on a bus error.
//
// This is the reason the FIFO exists: the SD card and NOR flash share SPI2
// with this sensor, so a card access can hold the bus for tens of
// milliseconds. Reading the data registers would simply lose every sample
// that arrived meanwhile; draining the FIFO delays them instead, and nothing
// is lost. Set *overrun if the FIFO wrapped, which means samples WERE lost.
int imu_read_fifo(imu_sample_t *out, int max, bool *overrun);

#endif // IMU_H

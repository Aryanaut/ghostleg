#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief MMC5983MA bandwidth settings
 */
typedef enum {
  MMC5983MA_BW_100HZ = 0, /*!< 100Hz bandwidth, 8ms measurement time */
  MMC5983MA_BW_200HZ = 1, /*!< 200Hz bandwidth, 4ms measurement time */
  MMC5983MA_BW_400HZ = 2, /*!< 400Hz bandwidth, 2ms measurement time */
  MMC5983MA_BW_800HZ = 3  /*!< 800Hz bandwidth, 0.5ms measurement time */
} mmc5983ma_bandwidth_t;

/**
 * @brief MMC5983MA continuous mode frequency settings
 */
typedef enum {
  MMC5983MA_FREQ_OFF = 0,   /*!< Continuous measurement disabled */
  MMC5983MA_FREQ_1HZ = 1,   /*!< 1Hz frequency */
  MMC5983MA_FREQ_10HZ = 2,  /*!< 10Hz frequency */
  MMC5983MA_FREQ_20HZ = 3,  /*!< 20Hz frequency */
  MMC5983MA_FREQ_50HZ = 4,  /*!< 50Hz frequency */
  MMC5983MA_FREQ_100HZ = 5, /*!< 100Hz frequency */
  MMC5983MA_FREQ_200HZ = 6, /*!< 200Hz frequency (BW=01 required) */
  MMC5983MA_FREQ_1000HZ = 7 /*!< 1000Hz frequency (BW=11 required) */
} mmc5983ma_frequency_t;

/**
 * @brief Magnetometer calibration data
 */
 typedef struct {
  float hard_iron_offset[3];  // Hard iron offset vector (Vx, Vy, Vz)
  float soft_iron_matrix[9];  // 3x3 soft iron transformation matrix (stored in row-major order)
  float field_magnitude;      // Geomagnetic field strength
  bool is_calibrated;         // Flag indicating whether calibration data is valid
} mmc5983ma_calibration_t;

/**
 * @brief MMC5983MA magnetometer device structure
 */
typedef struct {
  i2c_master_dev_handle_t i2c_dev;     /*!< I2C device handle */
  uint16_t sensitivity;                /*!< Sensitivity: counts per Gauss */
  bool auto_set_reset;                 /*!< Auto SET/RESET enabled state */
  bool continuous_mode;                /*!< Continuous mode enabled state */
  mmc5983ma_frequency_t frequency;     /*!< Continuous mode frequency */
  bool initialized;                    /*!< Initialization flag */
  mmc5983ma_calibration_t calibration; /*!< Calibration data */

  int32_t x_off, y_off, z_off;
} mmc5983ma_t;

/**
 * @brief Initialize the MMC5983MA magnetometer
 *
 * @param dev Pointer to the device structure
 * @param i2c_bus The I2C bus handle to use
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_init(mmc5983ma_t *dev, i2c_master_bus_handle_t i2c_bus);

/**
 * @brief Deinitialize the MMC5983MA magnetometer
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_deinit(mmc5983ma_t *dev);

/**
 * @brief Set the measurement bandwidth
 *
 * @param dev Pointer to the device structure
 * @param bandwidth The bandwidth/measurement time setting
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_set_bandwidth(mmc5983ma_t *dev,
                                  mmc5983ma_bandwidth_t bandwidth);

/**
 * @brief Enable continuous measurement mode
 *
 * @param dev Pointer to the device structure
 * @param frequency The measurement frequency
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_enable_continuous_mode(mmc5983ma_t *dev,
                                           mmc5983ma_frequency_t frequency);

/**
 * @brief Disable continuous measurement mode
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_disable_continuous_mode(mmc5983ma_t *dev);

/**
 * @brief Enable or disable automatic SET/RESET operations
 *
 * @param dev Pointer to the device structure
 * @param enable True to enable auto SET/RESET, false to disable
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_enable_auto_set_reset(mmc5983ma_t *dev, bool enable);

/**
 * @brief Software reset the device
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_software_reset(mmc5983ma_t *dev);

/**
 * @brief Read raw magnetometer data
 *
 * @param dev Pointer to the device structure
 * @param x Pointer to store X-axis raw value
 * @param y Pointer to store Y-axis raw value
 * @param z Pointer to store Z-axis raw value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_read_mag_raw(mmc5983ma_t *dev, int32_t *x, int32_t *y,
                                 int32_t *z);

/**
 * @brief Read magnetometer data in microTesla
 *
 * @param dev Pointer to the device structure
 * @param x Pointer to store X-axis value in uT
 * @param y Pointer to store Y-axis value in uT
 * @param z Pointer to store Z-axis value in uT
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_read_mag_ut(mmc5983ma_t *dev, float *x, float *y, float *z);

/**
 * @brief Read temperature data
 *
 * @param dev Pointer to the device structure
 * @param temperature Pointer to store temperature in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_read_temperature(mmc5983ma_t *dev, float *temperature);

/**
 * @brief Perform a SET operation
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_set(mmc5983ma_t *dev);

/**
 * @brief Perform a RESET operation
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_reset(mmc5983ma_t *dev);

/**
 * @brief Perform a calibrated measurement using SET/RESET technique
 *
 * This provides more accurate results by eliminating sensor offset
 *
 * @param dev Pointer to the device structure
 * @param x Pointer to store X-axis value in uT
 * @param y Pointer to store Y-axis value in uT
 * @param z Pointer to store Z-axis value in uT
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_set_reset_measurement(mmc5983ma_t *dev, float *x, float *y,
                                          float *z);

/**
 * @brief Get the product ID
 *
 * @param dev Pointer to the device structure
 * @param product_id Pointer to store the product ID
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_get_product_id(mmc5983ma_t *dev, uint8_t *product_id);

/**
 * @brief Check if a measurement is complete
 *
 * @param dev Pointer to the device structure
 * @param done Pointer to store completion status (true if done)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_check_measurement_done(mmc5983ma_t *dev, bool *done);

/**
 * @brief Clear the measurement done status flag
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mmc5983ma_clear_measurement_done(mmc5983ma_t *dev);

/**
 * @brief Perform magnetometer calibration using collected samples
 *
 * @param dev Pointer to the MMC5983MA device structure
 * @param samples Pointer to array of magnetometer samples (x,y,z values in uT)
 * @param num_samples Number of samples
 * @return esp_err_t ESP_OK on success
 */
 esp_err_t mmc5983ma_perform_calibration(mmc5983ma_t *dev, float (*samples)[3], int num_samples);

/**
* @brief Apply calibration to magnetometer reading
*
* @param dev Pointer to the MMC5983MA device structure
* @param raw_data Raw magnetometer data [x,y,z] in uT
* @param calibrated_data Calibrated magnetometer data [x,y,z] in uT
* @return float Quality metric (0.0-1.0) where 1.0 is perfect calibration
*/
float mmc5983ma_apply_calibration(const mmc5983ma_t *dev, const float raw_data[3], float calibrated_data[3]);

/**
* @brief Load calibration data into magnetometer
*
* @param dev Pointer to the MMC5983MA device structure
* @param calibration_data Pointer to the calibration data to load
* @return esp_err_t ESP_OK on success
*/
esp_err_t mmc5983ma_load_calibration(mmc5983ma_t *dev, const mmc5983ma_calibration_t *calibration_data);

/**
* @brief Get calibration data from magnetometer
*
* @param dev Pointer to the MMC5983MA device structure
* @param calibration_data Pointer to store the calibration data
* @return esp_err_t ESP_OK on success
*/
esp_err_t mmc5983ma_get_calibration(const mmc5983ma_t *dev, mmc5983ma_calibration_t *calibration_data);



#endif /* MMC5983MA_H */
#include "mmc5983ma.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "matrix_utils.h"

#define TAG "MMC5983MA"

/* MMC5983MA I2C Address */
#define MMC5983MA_I2C_ADDR                0x30

/* Register Addresses */
#define MMC5983MA_REG_XOUT0               0x00  /* Xout [17:10] */
#define MMC5983MA_REG_XOUT1               0x01  /* Xout [9:2] */
#define MMC5983MA_REG_YOUT0               0x02  /* Yout [17:10] */
#define MMC5983MA_REG_YOUT1               0x03  /* Yout [9:2] */
#define MMC5983MA_REG_ZOUT0               0x04  /* Zout [17:10] */
#define MMC5983MA_REG_ZOUT1               0x05  /* Zout [9:2] */
#define MMC5983MA_REG_XYZOUT2             0x06  /* Xout[1:0], Yout[1:0], Zout[1:0] */
#define MMC5983MA_REG_TOUT                0x07  /* Temperature output */
#define MMC5983MA_REG_STATUS              0x08  /* Device status */
#define MMC5983MA_REG_CONTROL0            0x09  /* Control register 0 */
#define MMC5983MA_REG_CONTROL1            0x0A  /* Control register 1 */
#define MMC5983MA_REG_CONTROL2            0x0B  /* Control register 2 */
#define MMC5983MA_REG_CONTROL3            0x0C  /* Control register 3 */
#define MMC5983MA_REG_PRODUCT_ID          0x2F  /* Product ID */

/* Control Register 0 Bit Definitions */
#define MMC5983MA_TM_M                    0x01  /* Take magnetic measurement */
#define MMC5983MA_TM_T                    0x02  /* Take temperature measurement */
#define MMC5983MA_INT_MEAS_DONE_EN        0x04  /* Measurement done interrupt enable */
#define MMC5983MA_SET                     0x08  /* SET operation */
#define MMC5983MA_RESET                   0x10  /* RESET operation */
#define MMC5983MA_AUTO_SR_EN              0x20  /* Automatic SET/RESET enable */
#define MMC5983MA_OTP_READ                0x40  /* OTP read */

/* Control Register 1 Bit Definitions */
#define MMC5983MA_BW0                     0x01  /* Bandwidth selection bit 0 */
#define MMC5983MA_BW1                     0x02  /* Bandwidth selection bit 1 */
#define MMC5983MA_X_INHIBIT               0x04  /* X channel inhibit */
#define MMC5983MA_YZ_INHIBIT              0x08  /* Y and Z channels inhibit */
#define MMC5983MA_SW_RST                  0x80  /* Software reset */

/* Control Register 2 Bit Definitions */
#define MMC5983MA_CMM_EN                  0x08  /* Continuous mode enable */
#define MMC5983MA_EN_PRD_SET              0x80  /* Enable periodic set */

/* Status Register Bit Definitions */
#define MMC5983MA_MEAS_M_DONE             0x01  /* Magnetic measurement done */
#define MMC5983MA_MEAS_T_DONE             0x02  /* Temperature measurement done */
#define MMC5983MA_OTP_READ_DONE           0x10  /* OTP read done */

/* Other Constants */
#define MMC5983MA_COUNTS_PER_GAUSS_16BIT  4096  /* 16-bit mode counts per Gauss */
#define MMC5983MA_COUNTS_PER_GAUSS_18BIT  16384 /* 18-bit mode counts per Gauss */
#define MMC5983MA_TEMPERATURE_OFFSET      75.0f /* Temperature offset in degrees C */
#define MMC5983MA_TEMPERATURE_SCALE       0.8f  /* Temperature scale in degrees C per LSB */
#define MMC5983MA_EXPECTED_PRODUCT_ID     0x30  /* Expected product ID value */

/* Private functions prototypes */
static esp_err_t mmc5983ma_i2c_read_reg(mmc5983ma_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t mmc5983ma_i2c_write_reg(mmc5983ma_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);

/* Private functions */
static esp_err_t mmc5983ma_i2c_read_reg(mmc5983ma_t *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (!dev || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return i2c_master_transmit_receive(dev->i2c_dev, &reg_addr, 1, data, len, -1);
}

static esp_err_t mmc5983ma_i2c_write_reg(mmc5983ma_t *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (!dev || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t *write_buf = (uint8_t *)malloc(len + 1);
    if (!write_buf) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);
    
    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, write_buf, len + 1, -1);
    
    free(write_buf);
    return ret;
}

esp_err_t mmc5983ma_init(mmc5983ma_t *dev, i2c_master_bus_handle_t i2c_bus) {
    if (!dev || !i2c_bus) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing MMC5983MA magnetometer");
    
    // Create I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MMC5983MA_I2C_ADDR,
        .scl_speed_hz = 400000,  // 400 kHz
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Check product ID
    uint8_t product_id;
    ret = mmc5983ma_get_product_id(dev, &product_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read product ID: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->i2c_dev);
        return ret;
    }
    
    if (product_id != MMC5983MA_EXPECTED_PRODUCT_ID) {
        ESP_LOGE(TAG, "Invalid product ID: 0x%02x, expected: 0x%02x", product_id, MMC5983MA_EXPECTED_PRODUCT_ID);
        i2c_master_bus_rm_device(dev->i2c_dev);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Perform a software reset
    ret = mmc5983ma_software_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->i2c_dev);
        return ret;
    }
    
    // Set default sensitivity for 18-bit mode
    dev->sensitivity = MMC5983MA_COUNTS_PER_GAUSS_18BIT;
    dev->auto_set_reset = false;
    dev->continuous_mode = false;      // Initialize continuous mode flag
    dev->frequency = MMC5983MA_FREQ_OFF; // Initialize frequency setting
    dev->x_off = 131072;
    dev->y_off = 131072;
    dev->z_off = 131072;

    // Mark device as initialized first
    dev->initialized = true;
    
    // Set bandwidth to default (BW=00, 100Hz)
    ret = mmc5983ma_set_bandwidth(dev, MMC5983MA_BW_100HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set bandwidth: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->initialized = false;
        return ret;
    }
    
    // Perform initial SET operation
    ret = mmc5983ma_set(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform SET operation: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->initialized = false;
        return ret;
    }

    // Initialize calibration data
    memset(&dev->calibration, 0, sizeof(mmc5983ma_calibration_t));
    // Set identity matrix for soft-iron correction
    eye(dev->calibration.soft_iron_matrix, 3);
    dev->calibration.is_calibrated = false;

    ESP_LOGI(TAG, "Calibration initialized with default values");
    ESP_LOGI(TAG, "MMC5983MA initialized successfully");
    return ESP_OK;
}

esp_err_t mmc5983ma_deinit(mmc5983ma_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Disable continuous mode if enabled
    mmc5983ma_disable_continuous_mode(dev);
    
    esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    dev->initialized = false;
    ESP_LOGI(TAG, "MMC5983MA deinitialized");
    return ESP_OK;
}

esp_err_t mmc5983ma_set_bandwidth(mmc5983ma_t *dev, mmc5983ma_bandwidth_t bandwidth) {
    if (!dev || bandwidth > MMC5983MA_BW_800HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Read current CONTROL1 register
    uint8_t control1;
    esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_CONTROL1, &control1, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONTROL1 register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear BW bits and set new value
    control1 &= ~(MMC5983MA_BW0 | MMC5983MA_BW1);
    control1 |= (uint8_t)bandwidth;
    
    // Write back to CONTROL1 register
    ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL1, &control1, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONTROL1 register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set bandwidth to %d", bandwidth);
    return ESP_OK;
}

esp_err_t mmc5983ma_enable_continuous_mode(mmc5983ma_t *dev, mmc5983ma_frequency_t frequency) {
    if (!dev || !dev->initialized || frequency == MMC5983MA_FREQ_OFF || frequency > MMC5983MA_FREQ_1000HZ) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if bandwidth settings are compatible with requested frequency
    if (frequency == MMC5983MA_FREQ_200HZ || frequency == MMC5983MA_FREQ_1000HZ) {
        uint8_t control1;
        esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_CONTROL1, &control1, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read CONTROL1 register: %s", esp_err_to_name(ret));
            return ret;
        }
        
        uint8_t bw = control1 & (MMC5983MA_BW0 | MMC5983MA_BW1);
        mmc5983ma_bandwidth_t required_bw = (frequency == MMC5983MA_FREQ_200HZ) ? 
                                           MMC5983MA_BW_200HZ : MMC5983MA_BW_800HZ;
        
        if (bw < required_bw) {
            ESP_LOGW(TAG, "Setting bandwidth to %d for requested frequency", required_bw);
            ret = mmc5983ma_set_bandwidth(dev, required_bw);
            if (ret != ESP_OK) {
                return ret;
            }
        }
    }
    
    // Set the measurement frequency and enable continuous mode in CONTROL2
    uint8_t control2 = (uint8_t)frequency | MMC5983MA_CMM_EN;
    
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL2, &control2, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable continuous mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // If auto_set_reset is enabled, configure periodic SET operation
    if (dev->auto_set_reset) {
        // Read current CONTROL2 to get existing bits
        ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_CONTROL2, &control2, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read CONTROL2 register: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Set En_prd_set bit and Prd_set to 010 (75 measurements)
        control2 |= MMC5983MA_EN_PRD_SET;          // Enable periodic SET
        control2 &= ~0x70;                         // Clear Prd_set bits
        control2 |= (2 << 4);                      // Set Prd_set to 010 (75 measurements)
        
        ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL2, &control2, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure periodic SET: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "Configured periodic SET for every 75 measurements");
    }
    
    // Update the struct flags
    dev->continuous_mode = true;
    dev->frequency = frequency;
    
    ESP_LOGI(TAG, "Enabled continuous mode with frequency %d", frequency);
    return ESP_OK;
}

esp_err_t mmc5983ma_disable_continuous_mode(mmc5983ma_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear CONTROL2 register to disable continuous mode
    uint8_t control2 = 0;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL2, &control2, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable continuous mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update the struct flags
    dev->continuous_mode = false;
    dev->frequency = MMC5983MA_FREQ_OFF;
    
    ESP_LOGI(TAG, "Disabled continuous mode");
    return ESP_OK;
}

esp_err_t mmc5983ma_enable_auto_set_reset(mmc5983ma_t *dev, bool enable) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read current CONTROL0 register
    uint8_t control0;
    esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONTROL0 register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set or clear AUTO_SR_EN bit
    if (enable) {
        control0 |= MMC5983MA_AUTO_SR_EN;
    } else {
        control0 &= ~MMC5983MA_AUTO_SR_EN;
    }
    
    // Write back to CONTROL0 register
    ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONTROL0 register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    dev->auto_set_reset = enable;
    ESP_LOGI(TAG, "%s auto SET/RESET", enable ? "Enabled" : "Disabled");
    return ESP_OK;
}

esp_err_t mmc5983ma_software_reset(mmc5983ma_t *dev) {
    if (!dev || !dev->i2c_dev) {  // Allow reset even if not initialized
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set SW_RST bit in CONTROL1
    uint8_t control1 = MMC5983MA_SW_RST;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL1, &control1, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONTROL1 register for reset: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for reset to complete (10ms according to datasheet)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Software reset completed");
    return ESP_OK;
}

esp_err_t mmc5983ma_read_mag_raw(mmc5983ma_t *dev, int32_t *x, int32_t *y, int32_t *z) {
    if (!dev || !dev->initialized || !x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use the continuous_mode flag from the struct instead of reading the register
    if (!dev->continuous_mode) {
        // Trigger a measurement
        uint8_t control0 = MMC5983MA_TM_M;
        esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to trigger measurement: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Wait for measurement to complete
        bool done = false;
        uint8_t retries = 0;
        const uint8_t max_retries = 20;  // Increased for reliability
        
        while (!done && retries < max_retries) {
            ret = mmc5983ma_check_measurement_done(dev, &done);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to check measurement status: %s", esp_err_to_name(ret));
                return ret;
            }
            
            if (!done) {
                vTaskDelay(pdMS_TO_TICKS(1));
                retries++;
            }
        }
        
        if (!done) {
            ESP_LOGE(TAG, "Measurement timed out");
            return ESP_ERR_TIMEOUT;
        }
    } else {
        // In continuous mode, we should check if new data is available
        bool done = false;
        esp_err_t ret = mmc5983ma_check_measurement_done(dev, &done);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check measurement status: %s", esp_err_to_name(ret));
            return ret;
        }
        
        if (!done) {
            ESP_LOGD(TAG, "Waiting for new measurement in continuous mode");
            uint8_t retries = 0;
            const uint8_t max_retries = 50;  // More retries for continuous mode
            
            while (!done && retries < max_retries) {
                ret = mmc5983ma_check_measurement_done(dev, &done);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to check measurement status: %s", esp_err_to_name(ret));
                    return ret;
                }
                
                if (!done) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    retries++;
                }
            }
            
            if (!done) {
                ESP_LOGW(TAG, "No new data available in continuous mode");
                // Continue anyway to read the last measurement
            }
        }
    }
    
    // Read all 7 bytes at once
    uint8_t data[7];
    esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_XOUT0, data, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Extract 18-bit values
    *x = ((int32_t)data[0] << 10) | ((int32_t)data[1] << 2) | ((data[6] >> 6) & 0x03);
    *y = ((int32_t)data[2] << 10) | ((int32_t)data[3] << 2) | ((data[6] >> 4) & 0x03);
    *z = ((int32_t)data[4] << 10) | ((int32_t)data[5] << 2) | ((data[6] >> 2) & 0x03);
    
    // Clear measurement done status
    ret = mmc5983ma_clear_measurement_done(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear measurement done status: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t mmc5983ma_read_mag_ut(mmc5983ma_t *dev, float *x, float *y, float *z) {
    if (!dev || !dev->initialized || !x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int32_t raw_x, raw_y, raw_z;
    esp_err_t ret = mmc5983ma_read_mag_raw(dev, &raw_x, &raw_y, &raw_z);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert from raw counts to microtesla
    // 1 Gauss = 100 uT
    *x = ((float)(raw_x - dev->x_off)) / (float)dev->sensitivity * 100.0f;
    *y = ((float)(raw_y - dev->y_off)) / (float)dev->sensitivity * 100.0f;
    *z = ((float)(raw_z - dev->z_off)) / (float)dev->sensitivity * 100.0f;
    
    return ESP_OK;
}

esp_err_t mmc5983ma_read_temperature(mmc5983ma_t *dev, float *temperature) {
    if (!dev || !dev->initialized || !temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Trigger a temperature measurement
    uint8_t control0 = MMC5983MA_TM_T;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger temperature measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for temperature measurement to complete
    bool done = false;
    uint8_t retries = 0;
    const uint8_t max_retries = 10;
    
    while (!done && retries < max_retries) {
        // Check if temperature measurement is done
        uint8_t status;
        ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_STATUS, &status, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
            return ret;
        }
        
        if (status & MMC5983MA_MEAS_T_DONE) {
            done = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
            retries++;
        }
    }
    
    if (!done) {
        ESP_LOGE(TAG, "Temperature measurement timed out");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read temperature register
    uint8_t temp_raw;
    ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_TOUT, &temp_raw, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert to degrees Celsius (datasheet formula)
    // Range is -75°C to 125°C, with 0.8°C/LSB, 0x00 = -75°C
    *temperature = (float)temp_raw * MMC5983MA_TEMPERATURE_SCALE - MMC5983MA_TEMPERATURE_OFFSET;
    
    // Clear temperature measurement done flag
    uint8_t clear_status = MMC5983MA_MEAS_T_DONE;
    ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_STATUS, &clear_status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear temperature done status: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t mmc5983ma_set(mmc5983ma_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Trigger a SET operation
    uint8_t control0 = MMC5983MA_SET;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger SET operation: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for SET to complete (at least 500ns according to datasheet, we use 1ms to be safe)
    vTaskDelay(pdMS_TO_TICKS(1));
    // esp_rom_delay_us(1);
    
    return ESP_OK;
}

esp_err_t mmc5983ma_reset(mmc5983ma_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Trigger a RESET operation
    uint8_t control0 = MMC5983MA_RESET;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_CONTROL0, &control0, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger RESET operation: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for RESET to complete (at least 500ns according to datasheet, we use 1ms to be safe)
    vTaskDelay(pdMS_TO_TICKS(1));
    // esp_rom_delay_us(1);
    
    return ESP_OK;
}

esp_err_t mmc5983ma_set_reset_measurement(mmc5983ma_t *dev, float *x, float *y, float *z) {
    if (!dev || !dev->initialized || !x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 1. Perform SET
    esp_err_t ret = mmc5983ma_set(dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 2. Read measurement after SET
    int32_t raw_x1, raw_y1, raw_z1;
    ret = mmc5983ma_read_mag_raw(dev, &raw_x1, &raw_y1, &raw_z1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 3. Perform RESET
    ret = mmc5983ma_reset(dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 4. Read measurement after RESET
    int32_t raw_x2, raw_y2, raw_z2;
    ret = mmc5983ma_read_mag_raw(dev, &raw_x2, &raw_y2, &raw_z2);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mmc5983ma_set(dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 5. Calculate field values using SET/RESET technique
    // H = (Output1 - Output2) / 2
    int32_t field_x = (raw_x1 - raw_x2) / 2;
    int32_t field_y = (raw_y1 - raw_y2) / 2;
    int32_t field_z = (raw_z1 - raw_z2) / 2;

    // store offsets for mmc5983ma_read_mag_ut
    dev->x_off = (raw_x1 + raw_x2) / 2;
    dev->y_off = (raw_y1 + raw_y2) / 2;
    dev->z_off = (raw_z1 + raw_z2) / 2;
    
    // Convert to microtesla (1 Gauss = 100 uT)
    *x = (float)field_x / (float)dev->sensitivity * 100.0f;
    *y = (float)field_y / (float)dev->sensitivity * 100.0f;
    *z = (float)field_z / (float)dev->sensitivity * 100.0f;
    
    return ESP_OK;
}

esp_err_t mmc5983ma_get_product_id(mmc5983ma_t *dev, uint8_t *product_id) {
    if (!dev || !product_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_PRODUCT_ID, product_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read product ID: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t mmc5983ma_check_measurement_done(mmc5983ma_t *dev, bool *done) {
    if (!dev || !dev->initialized || !done) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read status register
    uint8_t status;
    esp_err_t ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Check if measurement done flag is set
    *done = (status & MMC5983MA_MEAS_M_DONE) != 0;
    
    return ESP_OK;
}

esp_err_t mmc5983ma_clear_measurement_done(mmc5983ma_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Writing 1 to the MEAS_M_DONE bit clears the flag
    uint8_t clear_status = MMC5983MA_MEAS_M_DONE;
    esp_err_t ret = mmc5983ma_i2c_write_reg(dev, MMC5983MA_REG_STATUS, &clear_status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear measurement done status: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t mmc5983ma_perform_calibration(mmc5983ma_t *dev, float (*samples)[3], int num_samples) {
    if (dev == NULL || samples == NULL || num_samples < 10) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Performing calibration with %d samples", num_samples);

    // Find min/max for each axis as an initial hard-iron calibration
    float min_vals[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    float max_vals[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

    for (int i = 0; i < num_samples; i++) {
        float x_val = samples[i][0];
        float y_val = samples[i][1];
        float z_val = samples[i][2];

        if (x_val < min_vals[0]) min_vals[0] = x_val;
        if (x_val > max_vals[0]) max_vals[0] = x_val;

        if (y_val < min_vals[1]) min_vals[1] = y_val;
        if (y_val > max_vals[1]) max_vals[1] = y_val;

        if (z_val < min_vals[2]) min_vals[2] = z_val;
        if (z_val > max_vals[2]) max_vals[2] = z_val;
    }

    // Calculate initial hard-iron offsets (center of min/max)
    float hard_iron[3];
    for (int axis = 0; axis < 3; axis++) {
        hard_iron[axis] = (min_vals[axis] + max_vals[axis]) / 2.0f;
    }

    // Allocate centered_data on the heap
    float (*centered_data)[3] = malloc(num_samples * sizeof(*centered_data));
    if (centered_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for centered_data");
        return ESP_ERR_NO_MEM;
    }

    // Center the data by subtracting the hard-iron offsets
    for (int i = 0; i < num_samples; i++) {
        centered_data[i][0] = samples[i][0] - hard_iron[0];
        centered_data[i][1] = samples[i][1] - hard_iron[1];
        centered_data[i][2] = samples[i][2] - hard_iron[2];
    }

    // Compute covariance matrix (3x3 symmetric matrix)
    float covariance[9] = {0};
    for (int i = 0; i < num_samples; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                covariance[j * 3 + k] += centered_data[i][j] * centered_data[i][k];
            }
        }
    }
    
    // Normalize by number of samples
    matscale(covariance, covariance, 1.0f / num_samples, 3, 3);

    // Ensure matrix is exactly symmetric (handles numerical errors)
    symmetrize(covariance, 3);

    // Compute eigenvalues and eigenvectors of the covariance matrix
    float eigenvalues[3];
    float eigenvectors[9];
    symmetric_eigen_3x3(covariance, eigenvalues, eigenvectors, 50);

    // Check eigenvalues - they should all be positive for a valid ellipsoid
    if (eigenvalues[0] <= 0 || eigenvalues[1] <= 0 || eigenvalues[2] <= 0) {
        ESP_LOGE(TAG, "Invalid eigenvalues: [%.2f, %.2f, %.2f]",
                 eigenvalues[0], eigenvalues[1], eigenvalues[2]);
        free(centered_data);
        return ESP_FAIL;
    }

    // Compute the square root of the eigenvalues
    float sqrt_eigenvalues[3];
    for (int i = 0; i < 3; i++) {
        sqrt_eigenvalues[i] = sqrtf(eigenvalues[i]);
    }

    // Calculate the average radius (field magnitude)
    float field_magnitude = (sqrt_eigenvalues[0] + sqrt_eigenvalues[1] + sqrt_eigenvalues[2]) / 3.0f;

    // Construct the soft-iron correction matrix
    float scaling_matrix[9] = {0};
    scaling_matrix[0] = field_magnitude / sqrt_eigenvalues[0];
    scaling_matrix[4] = field_magnitude / sqrt_eigenvalues[1];
    scaling_matrix[8] = field_magnitude / sqrt_eigenvalues[2];

    // Compute soft iron matrix as: eigenvectors * scaling_matrix * eigenvectors_transpose
    float temp_matrix[9];
    float soft_iron[9];
    float eigenvectors_transpose[9];

    // First multiply: temp = eigenvectors * scaling_matrix
    matmul(eigenvectors, scaling_matrix, temp_matrix, 3, 3, 3);

    // Transpose eigenvectors
    transpose(eigenvectors, eigenvectors_transpose, 3, 3);

    // Final multiply: soft_iron = temp * eigenvectors_transpose
    matmul(temp_matrix, eigenvectors_transpose, soft_iron, 3, 3, 3);

    // Save calibration parameters
    for (int i = 0; i < 3; i++) {
        dev->calibration.hard_iron_offset[i] = hard_iron[i];
    }
    for (int i = 0; i < 9; i++) {
        dev->calibration.soft_iron_matrix[i] = soft_iron[i];
    }
    dev->calibration.field_magnitude = field_magnitude;
    dev->calibration.is_calibrated = true;

    // Log calibration results
    char *buffer = malloc(256);
    if (buffer != NULL) {
        snprintf(buffer, 256,
                "Calibration successful\n"
                "Hard-iron offsets: [%.2f, %.2f, %.2f]\n"
                "Field magnitude: %.2f\n"
                "Eigenvalues: [%.2f, %.2f, %.2f]\n"
                "Soft-iron matrix:\n"
                "[%.4f, %.4f, %.4f]\n"
                "[%.4f, %.4f, %.4f]\n"
                "[%.4f, %.4f, %.4f]",
                hard_iron[0], hard_iron[1], hard_iron[2],
                field_magnitude,
                eigenvalues[0], eigenvalues[1], eigenvalues[2],
                soft_iron[0], soft_iron[1], soft_iron[2],
                soft_iron[3], soft_iron[4], soft_iron[5],
                soft_iron[6], soft_iron[7], soft_iron[8]);

        ESP_LOGI(TAG, "%s", buffer);
        free(buffer);
    }

    // Free the dynamically allocated memory
    free(centered_data);

    return ESP_OK;
}

float mmc5983ma_apply_calibration(const mmc5983ma_t *dev, const float raw_data[3], float calibrated_data[3]) {
    if (dev == NULL || raw_data == NULL || calibrated_data == NULL) {
        return 0.0f;
    }

    if (!dev->calibration.is_calibrated) {
        // If not calibrated, just copy the data
        ESP_LOGW(TAG, "Magnetometer not calibrated!");
        calibrated_data[0] = raw_data[0];
        calibrated_data[1] = raw_data[1];
        calibrated_data[2] = raw_data[2];
        return 0.0f;
    }

    // Calculate current field strength
    float centered_x = raw_data[0] - dev->calibration.hard_iron_offset[0];
    float centered_y = raw_data[1] - dev->calibration.hard_iron_offset[1];
    float centered_z = raw_data[2] - dev->calibration.hard_iron_offset[2];
    float current_field = sqrtf(centered_x * centered_x + centered_y * centered_y + centered_z * centered_z);

    // Calculate quality factor
    // Calculate relative error
    float relative_error = fabsf(current_field - dev->calibration.field_magnitude) / dev->calibration.field_magnitude;

    // Convert to quality factor using exponential decay
    // This gives 1.0 for perfect match and decays with increasing error
    float quality = expf(-relative_error * 5.0f);

    // Create a vector for the centered values
    float centered_vec[3] = {centered_x, centered_y, centered_z};
    
    // Apply soft-iron correction matrix
    matvecmul(dev->calibration.soft_iron_matrix, centered_vec, calibrated_data, 3, 3);

    return quality;
}

esp_err_t mmc5983ma_load_calibration(mmc5983ma_t *dev, const mmc5983ma_calibration_t *calibration_data) {
    if (dev == NULL || calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy calibration data to the magnetometer structure
    memcpy(&dev->calibration, calibration_data, sizeof(mmc5983ma_calibration_t));

    char *buffer = malloc(256);
    snprintf(buffer, 256,
        "Loaded calibration data:\n"
        "Hard-iron offsets: [%.2f, %.2f, %.2f]\n"
        "Field magnitude: %.2f\n"
        "Soft-iron matrix:\n"
        "[%.4f, %.4f, %.4f]\n"
        "[%.4f, %.4f, %.4f]\n"
        "[%.4f, %.4f, %.4f]",
        dev->calibration.hard_iron_offset[0], dev->calibration.hard_iron_offset[1], dev->calibration.hard_iron_offset[2],
        dev->calibration.field_magnitude,
        dev->calibration.soft_iron_matrix[0], dev->calibration.soft_iron_matrix[1], dev->calibration.soft_iron_matrix[2],
        dev->calibration.soft_iron_matrix[3], dev->calibration.soft_iron_matrix[4], dev->calibration.soft_iron_matrix[5],
        dev->calibration.soft_iron_matrix[6], dev->calibration.soft_iron_matrix[7], dev->calibration.soft_iron_matrix[8]);

    ESP_LOGI(TAG, "%s", buffer);
    free(buffer);

    return ESP_OK;
}

esp_err_t mmc5983ma_get_calibration(const mmc5983ma_t *dev, mmc5983ma_calibration_t *calibration_data) {
    if (dev == NULL || calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy calibration data from the magnetometer structure
    memcpy(calibration_data, &dev->calibration, sizeof(mmc5983ma_calibration_t));

    return ESP_OK;
}
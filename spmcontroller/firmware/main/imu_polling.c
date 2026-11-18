// LSM6DSV80X driver for the Training Board IMU bridge.
#include "lsm6dsv80x_reg.h"
#include "pins.h"
#include "imu.h"
#include "imu_link.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include <hal/spi_types.h>
#include <esp_err.h>
#include <esp_log.h>

#define TAG "imu"

static lsm6dsv80x_filt_settling_mask_t filt_settling_mask;
static spi_device_handle_t lsm6dsv80x_dev = NULL;
static stmdev_ctx_t dev_ctx;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void delay(uint32_t ms);

// NOTE: SPI bus must be already initialized before calling this function
static esp_err_t spi_device_init(uint8_t mode, int clock_speed_hz) {
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = clock_speed_hz,
        .mode = mode,
        .spics_io_num = SPI2_IMU_CS_PIN,
        .queue_size = 7,
        .command_bits = 0,
        .address_bits = 8,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    return spi_bus_add_device(SPI2_HOST, &dev_config, &lsm6dsv80x_dev);
}


esp_err_t imu_init(void) {
    /* Power-on settling delay for IMU internal regulator/boot */
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t ret;
    uint8_t whoamI = 0;
    const uint8_t modes_to_try[] = { 0, 3 };

    for (size_t m = 0; m < sizeof(modes_to_try); m++) {
        if (lsm6dsv80x_dev != NULL) {
            spi_bus_remove_device(lsm6dsv80x_dev);
            lsm6dsv80x_dev = NULL;
        }

        ret = spi_device_init(modes_to_try[m], IMU_SPI_CLOCK_HZ);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "IMU spi_bus_add_device (mode %d) failed: %s", modes_to_try[m], esp_err_to_name(ret));
            continue;
        }

        dev_ctx.write_reg = platform_write;
        dev_ctx.read_reg = platform_read;
        dev_ctx.mdelay = delay;
        dev_ctx.handle = lsm6dsv80x_dev;

        whoamI = 0;
        lsm6dsv80x_device_id_get(&dev_ctx, &whoamI);
        ESP_LOGI(TAG, "SPI Mode %d -> LSM6DSV80X_ID: 0x%02X (expected 0x%02X)", modes_to_try[m], whoamI, LSM6DSV80X_ID);
        if (whoamI == LSM6DSV80X_ID || whoamI == 0x70) {
            ESP_LOGI(TAG, "LSM6DSV80X detected on SPI Mode %d (ID: 0x%02X)", modes_to_try[m], whoamI);
            break;
        }
    }

    if (whoamI != LSM6DSV80X_ID && whoamI != 0x70) {
        ESP_LOGE(TAG, "IMU WHO_AM_I mismatch: expected 0x%X got 0x%X", LSM6DSV80X_ID, whoamI);
        return ESP_ERR_NOT_FOUND;
    }

    /* Perform device power-on-reset */
    lsm6dsv80x_sw_por(&dev_ctx);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Block Data Update: high/low bytes of a reading always come from the same sample */
    lsm6dsv80x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Output data rates. Nothing downstream uses more than IMU_LINK_RATE_HZ,
     * and every sample above that rate is bus time taken from the SD card and
     * NOR flash on the shared SPI2. 240 Hz is the part's nearest rate at or
     * above 200 Hz; the high-g accelerometer cannot go below 480 Hz, so it is
     * simply sampled at that rate and read on the 240 Hz data-ready. */
    lsm6dsv80x_xl_data_rate_set(&dev_ctx, LSM6DSV80X_ODR_AT_240Hz);
    lsm6dsv80x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV80X_HG_XL_ODR_AT_480Hz, 1);
    lsm6dsv80x_gy_data_rate_set(&dev_ctx, LSM6DSV80X_ODR_AT_240Hz);

    /* Full scale */
    lsm6dsv80x_xl_full_scale_set(&dev_ctx, LG_ACCEL_FULL_SCALE);
    lsm6dsv80x_hg_xl_full_scale_set(&dev_ctx, HG_ACCEL_FULL_SCALE);
    lsm6dsv80x_gy_full_scale_set(&dev_ctx, GYRO_FULL_SCALE);

    /* Filtering chain (same as Avionics C onboard config) */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv80x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    lsm6dsv80x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV80X_GY_ULTRA_LIGHT);
    lsm6dsv80x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV80X_XL_ULTRA_LIGHT);

    /* FIFO: batch accelerometer and gyro at the link rate, and the high-g
     * accelerometer at its own (minimum) 480 Hz. Continuous mode, so if we
     * ever fall far enough behind to fill 4 KB the OLDEST samples are dropped
     * and the newest survive. */
    lsm6dsv80x_fifo_xl_batch_set(&dev_ctx, LSM6DSV80X_XL_BATCHED_AT_240Hz);
    lsm6dsv80x_fifo_gy_batch_set(&dev_ctx, LSM6DSV80X_GY_BATCHED_AT_240Hz);
    lsm6dsv80x_fifo_hg_xl_batch_set(&dev_ctx, 1);
    lsm6dsv80x_fifo_mode_set(&dev_ctx, LSM6DSV80X_STREAM_MODE);

    /* Data-ready on INT1 as a pulse per new accelerometer sample. Gyro and
     * high-g run at the same ODR and are read in the same burst. */
    lsm6dsv80x_data_ready_mode_set(&dev_ctx, LSM6DSV80X_DRDY_PULSED);
    lsm6dsv80x_pin_int_route_t int1_route;
    memset(&int1_route, 0, sizeof(int1_route));
    int1_route.drdy_xl = PROPERTY_ENABLE;
    lsm6dsv80x_pin_int1_route_set(&dev_ctx, &int1_route);

    /* SPI2 is shared with the SD card and NOR flash; every read arbitrates
     * for the bus (polling transfers, ~30 us when the bus is free). */

    ESP_LOGI(TAG, "IMU configured: %d Hz xl/gy (480 Hz hg), FS 16g/64g/2000dps, INT1 drdy pulsed", IMU_LINK_RATE_HZ);
    return ESP_OK;
}

esp_err_t imu_read_sample(imu_sample_t *sample) {
    // Burst 1: STATUS_REG (0x1E), 0x1F, OUT_TEMP (0x20-21), OUTX..Z_G (0x22-27), OUTX..Z_A (0x28-2D)
    uint8_t b1[16];
    // Burst 2: UI_OUTX..Z_L_A_HG (0x34-39)
    uint8_t b2[6];

    if (platform_read(lsm6dsv80x_dev, LSM6DSV80X_STATUS_REG, b1, sizeof(b1)) != 0) return ESP_FAIL;

    lsm6dsv80x_status_reg_t status;
    memcpy(&status, &b1[0], 1);

    uint8_t flags = 0;
    if (status.xlda) flags |= IMU_LINK_FLAG_XL;
    if (status.gda) flags |= IMU_LINK_FLAG_GY;
    if (status.tda) flags |= IMU_LINK_FLAG_TEMP;
    if (status.xlhgda) flags |= IMU_LINK_FLAG_HG;

    memset(sample, 0, sizeof(*sample));
    sample->flags = flags;
    if (flags == 0) return ESP_OK;

    sample->temp  = (int16_t)((uint16_t)b1[2] | ((uint16_t)b1[3] << 8));
    sample->gy[0] = (int16_t)((uint16_t)b1[4] | ((uint16_t)b1[5] << 8));
    sample->gy[1] = (int16_t)((uint16_t)b1[6] | ((uint16_t)b1[7] << 8));
    sample->gy[2] = (int16_t)((uint16_t)b1[8] | ((uint16_t)b1[9] << 8));
    sample->xl[0] = (int16_t)((uint16_t)b1[10] | ((uint16_t)b1[11] << 8));
    sample->xl[1] = (int16_t)((uint16_t)b1[12] | ((uint16_t)b1[13] << 8));
    sample->xl[2] = (int16_t)((uint16_t)b1[14] | ((uint16_t)b1[15] << 8));

    if (flags & IMU_LINK_FLAG_HG) {
        if (platform_read(lsm6dsv80x_dev, LSM6DSV80X_UI_OUTX_L_A_HG, b2, sizeof(b2)) != 0) return ESP_FAIL;
        sample->hg[0] = (int16_t)((uint16_t)b2[0] | ((uint16_t)b2[1] << 8));
        sample->hg[1] = (int16_t)((uint16_t)b2[2] | ((uint16_t)b2[3] << 8));
        sample->hg[2] = (int16_t)((uint16_t)b2[4] | ((uint16_t)b2[5] << 8));
    }
    return ESP_OK;
}

// ---- FIFO drain ------------------------------------------------------------
// Each FIFO word is 7 bytes: a tag byte then 6 data bytes. The tag byte packs
// the sensor id in bits 7..3 and a 2-bit batch counter in bits 2..1.
#define FIFO_WORD_LEN     7
#define FIFO_TAG_GY       0x01
#define FIFO_TAG_XL       0x02
#define FIFO_TAG_XL_HG    0x1D
// Words per SPI transaction. 48 words is 336 bytes, about 270 us at 10 MHz -
// short enough that draining never becomes the thing that blocks the bus.
#define FIFO_BURST_WORDS  48

// A sample under construction, kept across calls because a batch can straddle
// the boundary between two drains.
static imu_sample_t s_partial;
static bool s_have_xl = false, s_have_gy = false;

static inline int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

int imu_read_fifo(imu_sample_t *out, int max, bool *overrun) {
    lsm6dsv80x_fifo_status_t st;
    if (lsm6dsv80x_fifo_status_get(&dev_ctx, &st) != 0) return -1;
    if (overrun) *overrun = st.fifo_ovr ? true : false;

    uint16_t level = st.fifo_level;
    int n_out = 0;
    static uint8_t buf[FIFO_BURST_WORDS * FIFO_WORD_LEN];

    while (level > 0 && n_out < max) {
        uint16_t words = level > FIFO_BURST_WORDS ? FIFO_BURST_WORDS : level;
        if (platform_read(lsm6dsv80x_dev, LSM6DSV80X_FIFO_DATA_OUT_TAG, buf, words * FIFO_WORD_LEN) != 0) {
            return n_out > 0 ? n_out : -1;
        }
        level -= words;

        for (uint16_t i = 0; i < words && n_out < max; i++) {
            const uint8_t *w = &buf[i * FIFO_WORD_LEN];
            switch (w[0] >> 3) {
                case FIFO_TAG_XL:
                    s_partial.xl[0] = le16(&w[1]); s_partial.xl[1] = le16(&w[3]); s_partial.xl[2] = le16(&w[5]);
                    s_have_xl = true;
                    break;
                case FIFO_TAG_GY:
                    s_partial.gy[0] = le16(&w[1]); s_partial.gy[1] = le16(&w[3]); s_partial.gy[2] = le16(&w[5]);
                    s_have_gy = true;
                    break;
                case FIFO_TAG_XL_HG:
                    // Batched at 480 Hz against a 240 Hz link, so the newer of
                    // each pair is the one that ships. The full-rate high-g
                    // stream stays in this board's own log if it is ever needed.
                    s_partial.hg[0] = le16(&w[1]); s_partial.hg[1] = le16(&w[3]); s_partial.hg[2] = le16(&w[5]);
                    s_partial.flags |= IMU_LINK_FLAG_HG;
                    break;
                default:
                    break;   // temperature, timestamp, config-change: not batched
            }
            // One accelerometer and one gyro reading make a complete sample.
            if (s_have_xl && s_have_gy) {
                s_partial.flags |= IMU_LINK_FLAG_XL | IMU_LINK_FLAG_GY;
                out[n_out++] = s_partial;
                s_have_xl = s_have_gy = false;
                s_partial.flags = 0;   // high-g re-latches on its next entry
            }
        }
    }
    return n_out;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    spi_transaction_t txn = {
        .flags = 0,
        .addr = reg & 0x7F, // Clear MSB for write operation
        .length = len * 8,
        .tx_buffer = (uint8_t *)bufp,
    };
    esp_err_t ret = spi_device_polling_transmit(handle, &txn);
    return ret == ESP_OK ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    spi_transaction_t txn = {
        .flags = 0,
        .addr = reg | 0x80, // Set MSB for read operation
        .rxlength = len * 8,
        .rx_buffer = bufp,
    };
    esp_err_t ret = spi_device_polling_transmit(handle, &txn);
    return ret == ESP_OK ? 0 : -1;
}

static void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

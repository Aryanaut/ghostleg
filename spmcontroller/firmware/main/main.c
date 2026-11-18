// Training Board firmware: IMU bridge + datalogger for Avionics C.
//
// The Avionics C onboard IMU is dead, so this board streams its own LSM6DSV80X
// over UART0 (J4 pin 15 "TX0") to Avionics C J4 pin 3 ("RX0") - see bridge.c
// and imu_link.h. It also logs IMU + barometer + magnetometer to its own SD
// card and NOR flash (tb_log.c) so the flight can be re-stitched if the UART
// link is lost: every record carries the timestamp of the last IMU frame sent.
//
// Boot: NVS -> SPI2 -> IMU -> NOR log -> session number + boot log -> SD ->
// dump NOR backlog to SD and wipe (only after a verified dump) -> session file
// -> bridge task (core 1, top priority) -> NOR/SD writer + sensor log tasks.
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include "lsm6dsv80x_reg.h"

#include "pins.h"
#include "imu.h"
#include "imu_link.h"
#include "bridge.h"
#include "sim_replay.h"
#include "tb_log.h"
#include "nor_log.h"
#include "storage.h"
#include "sdcard.h"
#include "sensor_data.h"
#include "barometer.h"
#include "magnetometer.h"
#include "usb_stream.h"

#define TAG "main"

#define STATS_PERIOD_MS        1000
#define IMU_REINIT_PERIOD_MS   2000

static const spi_bus_config_t spi_config = {
    .mosi_io_num = SPI2_MOSI_PIN,
    .miso_io_num = SPI2_MISO_PIN,
    .sclk_io_num = SPI2_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
};

static const i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C1_SDA_PIN,
    .scl_io_num = I2C1_SCL_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

// Hold every SPI2 chip select high before any driver touches the bus.
static void spi_cs_idle_init(void) {
    gpio_config_t cs = {
        .pin_bit_mask = (1ULL << SPI2_IMU_CS_PIN) | (1ULL << SPI2_NOR_FLASH_CS_PIN) | (1ULL << SPI2_SDC_CS_PIN),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs);
    gpio_set_level(SPI2_IMU_CS_PIN, 1);
    gpio_set_level(SPI2_NOR_FLASH_CS_PIN, 1);
    gpio_set_level(SPI2_SDC_CS_PIN, 1);
}

void app_main(void) {
    // SPM rig build: frame everything on the USB port from the very start, so
    // the PC can decode raw IMU/mag samples with the log text interleaved.
    if (usb_stream_init() != ESP_OK) {
        ESP_LOGE(TAG, "USB sensor stream failed to start - continuing without it");
    }
    ESP_LOGI(TAG, "=== Space Bar Training Board: IMU-over-UART bridge + datalogger for Avionics C ===");
#if SIM_REPLAY
    ESP_LOGW(TAG, "%s", sim_replay_describe());
    ESP_LOGW(TAG, "SIM REPLAY: IMU and barometer values are synthetic; the FIFO, the interrupt and the SPI bus are real, and link t_us carries sim time");
#endif
    ESP_LOGI(TAG, "protocol v%d, %d-byte frames, %d baud; log record %u bytes @ %d Hz",
             IMU_LINK_VERSION, IMU_LINK_PACKET_LEN, IMU_LINK_BAUD, (unsigned)sizeof(tb_log_record_t), 1000 / TB_LOG_PERIOD_MS);

    ESP_ERROR_CHECK(storage_nvs_init());
    spi_cs_idle_init();
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_config, SPI_DMA_CH_AUTO));

    // ---- SD card FIRST on the shared SPI2 bus ----
    // A freshly powered SD card is in native SD mode and can disturb the shared
    // CLK/MOSI/MISO lines while other devices talk; the SD init sequence
    // (74 clocks + CMD0) switches it to SPI mode, after which it ignores the
    // bus while its CS is high. Only then are the IMU and NOR flash probed.
    bool sd_ok = sdcard_init() == ESP_OK;

    // ---- NOR log ----
    nor_log_config_t nor_cfg = {
        .host = SPI2_HOST, .cs_gpio = SPI2_NOR_FLASH_CS_PIN, .cs_id = 1,
        .wp_gpio = FLASH_WP_PIN, .hold_gpio = FLASH_HOLD_PIN, .freq_mhz = 40,
    };
    bool nor_ok = nor_log_init(&nor_cfg) == ESP_OK;
    if (!nor_ok) ESP_LOGE(TAG, "NOR log init failed - logging to SD only");

    // ---- IMU (flight critical: keep retrying) ----
    while (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed, retrying in %d ms", IMU_REINIT_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(IMU_REINIT_PERIOD_MS));
    }

    // ---- session + boot log ----
    uint32_t session = storage_session_begin();
    ESP_LOGI(TAG, "session %lu", (unsigned long)session);
    storage_bootlog_begin(session, nor_ok, nor_ok ? nor_log_bytes_pending() : 0);

    // ---- NOR -> SD dump, session file ----
    FILE *sd_file = NULL;
    if (sd_ok) {
        storage_boot_dump_and_wipe(SDCARD_MOUNT_POINT, session);
#if TB_LOG_SD_LIVE
        sd_file = storage_open_session_file(SDCARD_MOUNT_POINT, &session, 0);
#else
        ESP_LOGI(TAG, "live SD logging off (TB_LOG_SD_LIVE=0): this session is written to NOR and dumped to the SD card at the next boot");
#endif
    } else {
        ESP_LOGE(TAG, "no SD card - NOR keeps its data until a boot with a working card");
        storage_bootlog_finish(false, false, false);
    }

    // ---- bridge first: nothing below may delay the IMU stream ----
    ESP_ERROR_CHECK(bridge_start());

    // ---- I2C sensors ----
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &i2c_bus));
    sensor_data_init();
    if (baro_init(i2c_bus) != ESP_OK)         ESP_LOGE(TAG, "barometer init failed");
    if (magnetometer_init(i2c_bus) != ESP_OK) ESP_LOGE(TAG, "magnetometer init failed");

    // ---- storage writer tasks + logger ----
    if (nor_ok) nor_log_task_start(3, 0);
    ESP_ERROR_CHECK(tb_log_start(sd_file));

    uint32_t last_sent = 0, last_rec = 0;
    int64_t last_stats_us = esp_timer_get_time();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(STATS_PERIOD_MS));
        int64_t now = esp_timer_get_time();
        bridge_stats_t st;
        bridge_get_stats(&st);
        bridge_snapshot_t snap;
        bridge_get_snapshot(&snap);
        uint32_t d_sent = st.packets_sent - last_sent;
        float dt_s = (now - last_stats_us) / 1e6f;
        float avg_us = d_sent ? (float)st.read_time_sum_us / d_sent : 0;
        const imu_sample_t *l = &snap.sample;

        ESP_LOGI(TAG, "tx %lu pkts (%.1f Hz) total=%lu | int=%lu poll=%lu empty=%lu partial=%lu spi_err=%lu uart_err=%lu | "
                      "isr->uart avg %.1f us max %lld us (>1ms: %lu, >5ms: %lu) | fifo max_drain=%lu overruns=%lu | last age %lld ms",
                 (unsigned long)d_sent, dt_s > 0 ? d_sent / dt_s : 0.f, (unsigned long)st.packets_sent,
                 (unsigned long)st.interrupts, (unsigned long)st.poll_fallbacks, (unsigned long)st.empty_reads,
                 (unsigned long)st.partial_samples, (unsigned long)st.spi_errors, (unsigned long)st.uart_errors,
                 avg_us, (long long)st.read_time_max_us, (unsigned long)st.reads_over_1ms, (unsigned long)st.reads_over_5ms,
                 (unsigned long)st.max_drain, (unsigned long)st.fifo_overruns,
                 (long long)((now - snap.sent_us) / 1000));
        ESP_LOGI(TAG, "last sample: xl[mg]=(%.0f, %.0f, %.0f) hg[mg]=(%.0f, %.0f, %.0f) gy[dps]=(%.2f, %.2f, %.2f) temp=%.1f C flags=0x%02x",
                 LG_ACCEL_TO_MG(l->xl[0]), LG_ACCEL_TO_MG(l->xl[1]), LG_ACCEL_TO_MG(l->xl[2]),
                 HG_ACCEL_TO_MG(l->hg[0]), HG_ACCEL_TO_MG(l->hg[1]), HG_ACCEL_TO_MG(l->hg[2]),
                 GYRO_TO_MDPS(l->gy[0]) / 1000.f, GYRO_TO_MDPS(l->gy[1]) / 1000.f, GYRO_TO_MDPS(l->gy[2]) / 1000.f,
                 lsm6dsv80x_from_lsb_to_celsius(l->temp), l->flags);
        SensorData sdat = sensor_data_get();
        uint32_t recs = tb_log_records();
        uint32_t nor_free = nor_log_ok() ? (nor_log_chip_size() - NOR_LOG_DATA_OFFSET - nor_log_bytes_pending()) : 0;
        float nor_minutes = (float)nor_free / (sizeof(tb_log_record_t) * (1000.0f / TB_LOG_PERIOD_MS)) / 60.0f;
        ESP_LOGI(TAG, "log: %.0f rec/s total=%lu | sd drops=%lu err=%lu max %lu us (fwrite %lu, fsync %lu, card-writes %lu) | nor rec=%lu drops=%lu err=%lu max_write %lu us pending=%lu KB free %lu KB (~%.0f min) %s buffered=%lu | baro %.2f hPa %.1f C | mag (%.1f, %.1f, %.1f) uT",
                 dt_s > 0 ? (recs - last_rec) / dt_s : 0.f, (unsigned long)recs,
                 (unsigned long)tb_log_sd_drops(), (unsigned long)tb_log_sd_write_errors(), (unsigned long)tb_log_sd_max_write_us(),
                 (unsigned long)tb_log_sd_flush_max_us(), (unsigned long)tb_log_sd_sync_max_us(), (unsigned long)tb_log_sd_block_writes(),
                 (unsigned long)nor_log_records_written(), (unsigned long)nor_log_drops(), (unsigned long)nor_log_write_errors(),
                 (unsigned long)nor_log_max_write_us(), (unsigned long)(nor_log_bytes_pending() / 1024),
                 (unsigned long)(nor_free / 1024), nor_minutes, nor_log_ok() ? (tb_log_launched() ? "LAUNCHED" : "armed") : "OFF", (unsigned long)tb_log_nor_buffered(),
                 sdat.baro_pressure, sdat.baro_temp, sdat.mag_x, sdat.mag_y, sdat.mag_z);
        last_sent = st.packets_sent;
        last_rec = recs;
        last_stats_us = now;
    }
}

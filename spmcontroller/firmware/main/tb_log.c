#include "tb_log.h"
#include "imu.h"
#include "imu_link.h"
#include "bridge.h"
#include "nor_log.h"
#include "sensor_data.h"
#include "barometer.h"
#include "magnetometer.h"

#include <string.h>
#include <unistd.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_crc.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define TAG "tb_log"

#define SD_QUEUE_DEPTH     256
// Records written per post-tick window. Must exceed the record rate divided by
// the window rate with margin; each is an 80-byte fwrite that usually lands in
// the stdio buffer and only occasionally reaches the card.
#define SD_RECORDS_PER_WINDOW  16
#define SD_FSYNC_MS        10000    // fsync is several random sector writes: keep it rare
// Write size. Measured on the flight card: a write costs roughly 100 ms almost
// regardless of size, because the card does a read-modify-write of a much
// larger internal page. Single-sector writes therefore cap the card at about
// 5 kB/s, below the 8.4 kB/s this board produces, and a third of all records
// were dropped for want of queue space. Large writes carry the same per-write
// cost and so are far cheaper per byte.
//
// The stall this creates is no longer the IMU's problem: since the sensor is
// read through its FIFO, samples that arrive while the bus is held are
// delivered late rather than lost.
#define SD_FILE_BUFFER     8192

// Pre-launch ring: the last TB_LOG_PRELAUNCH_MS of records, held in RAM so the
// NOR flash sees nothing until there is a flight to record.
#define PRELAUNCH_RECORDS  ((TB_LOG_PRELAUNCH_MS / TB_LOG_PERIOD_MS) + 1)
static tb_log_record_t s_ring[PRELAUNCH_RECORDS];
static uint32_t s_ring_head = 0, s_ring_count = 0;
static bool s_launched = false;
static uint8_t s_launch_run = 0;

static FILE *s_sd = NULL;
static QueueHandle_t s_sd_queue = NULL;
static volatile uint32_t s_records = 0, s_sd_drops = 0, s_sd_err = 0, s_sd_max_us = 0;
static volatile uint32_t s_sd_max_write_us = 0, s_sd_max_sync_us = 0, s_sd_writes = 0, s_sd_syncs = 0;

uint32_t tb_log_records(void)          { return s_records; }
bool     tb_log_launched(void)         { return s_launched; }
uint32_t tb_log_nor_buffered(void)     { return s_ring_count; }

// |low-g acceleration| in mg, from the raw LSB values in the record.
static uint32_t accel_magnitude_mg(const tb_log_record_t *r) {
    float x = r->xl[0] * 0.488f, y = r->xl[1] * 0.488f, z = r->xl[2] * 0.488f;
    return (uint32_t)sqrtf(x * x + y * y + z * z);
}

// Everything the NOR flash receives goes through here. On the pad the record
// only enters the ring; at launch the ring is drained in order and every
// record from then on goes straight through.
static void nor_offer(const tb_log_record_t *rec) {
    if (!s_launched) {
        s_ring[s_ring_head] = *rec;
        s_ring_head = (s_ring_head + 1) % PRELAUNCH_RECORDS;
        if (s_ring_count < PRELAUNCH_RECORDS) s_ring_count++;
        return;
    }
    nor_log_submit(rec, sizeof(*rec));
}

static void nor_drain_ring(void) {
    uint32_t start = (s_ring_head + PRELAUNCH_RECORDS - s_ring_count) % PRELAUNCH_RECORDS;
    for (uint32_t i = 0; i < s_ring_count; i++) {
        nor_log_submit(&s_ring[(start + i) % PRELAUNCH_RECORDS], sizeof(tb_log_record_t));
    }
    ESP_LOGW(TAG, "LAUNCH DETECTED - drained %lu buffered records (%lu ms of pad data) into the NOR flash; logging continuously from here",
             (unsigned long)s_ring_count, (unsigned long)(s_ring_count * TB_LOG_PERIOD_MS));
    s_ring_count = 0;
}
uint32_t tb_log_sd_drops(void)         { return s_sd_drops; }
uint32_t tb_log_sd_write_errors(void)  { return s_sd_err; }
uint32_t tb_log_sd_max_write_us(void)  { uint32_t v = s_sd_max_us; s_sd_max_us = 0; return v; }
uint32_t tb_log_sd_flush_max_us(void)  { uint32_t v = s_sd_max_write_us; s_sd_max_write_us = 0; return v; }
uint32_t tb_log_sd_sync_max_us(void)   { uint32_t v = s_sd_max_sync_us; s_sd_max_sync_us = 0; return v; }
uint32_t tb_log_sd_block_writes(void)  { return s_sd_writes; }

static void sd_writer_task(void *arg) {
    tb_log_record_t rec;
    int64_t last_sync = esp_timer_get_time();
    bool dirty = false;
    // Take the SPI bus only in the window just after an IMU frame has gone out.
    bridge_notify_after_tick(xTaskGetCurrentTaskHandle());
    while (1) {
        // Wait for that window. The timeout keeps the task alive (and the file
        // flushed) if the IMU ever stops ticking.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        // Drain several records per window, not one. The window opens once per
        // FIFO drain, and a drain covers several IMU samples, so the wake rate
        // is well below the 100 Hz record rate: writing a single record per
        // wake fell behind and the queue overflowed (7900 records dropped).
        for (int budget = SD_RECORDS_PER_WINDOW; budget > 0; budget--) {
        if (xQueueReceive(s_sd_queue, &rec, 0) == pdTRUE) {
            int64_t t0 = esp_timer_get_time();
            size_t w = fwrite(&rec, 1, sizeof(rec), s_sd);
            uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
            if (dt > s_sd_max_us) s_sd_max_us = dt;
            if (dt > s_sd_max_write_us) s_sd_max_write_us = dt;
            if (dt > 1000) s_sd_writes++;          // writes that actually reached the card
            if (w != sizeof(rec)) {
                s_sd_err++;
                if (s_sd_err == 1 || (s_sd_err % 100) == 0) ESP_LOGE(TAG, "SD short write (%u of %u bytes), errors=%lu", (unsigned)w, (unsigned)sizeof(rec), (unsigned long)s_sd_err);
            } else {
                dirty = true;
            }
        } else {
            break;   // queue empty; give the bus back until the next window
        }
        }
        int64_t now = esp_timer_get_time();
        // See TB_LOG_FSYNC_IN_FLIGHT / TB_LOG_FSYNC_ON_PAD. On the pad the FAT
        // metadata is written once a second so the file has a real length if
        // the power is pulled; after launch it is never written, because the
        // bus hold is not worth paying in flight and the NOR has the flight.
        bool sync_allowed = s_launched ? (bool)TB_LOG_FSYNC_IN_FLIGHT
                                       : (bool)TB_LOG_FSYNC_ON_PAD;
        if (dirty && sync_allowed && now - last_sync > (int64_t)TB_LOG_FSYNC_PERIOD_MS * 1000) {
            int64_t t0 = now;
            fflush(s_sd);
            fsync(fileno(s_sd));
            uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
            if (dt > s_sd_max_us) s_sd_max_us = dt;
            if (dt > s_sd_max_sync_us) s_sd_max_sync_us = dt;
            s_sd_syncs++;
            last_sync = now;
            dirty = false;
        }
    }
}

// Poll the I2C sensors and emit one record per period.
static void log_task(void *arg) {
    TickType_t wake = xTaskGetTickCount();
    tb_log_record_t rec;
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TB_LOG_PERIOD_MS));

        barometer_update();
        magnetometer_update();
        SensorData sd = sensor_data_get();

        bridge_snapshot_t b;
        bridge_get_snapshot(&b);

        memset(&rec, 0, sizeof(rec));
        rec.magic = TB_LOG_MAGIC;
        rec.len = sizeof(rec);
        rec.version = TB_LOG_VERSION;
        rec.t_us = (uint64_t)esp_timer_get_time();
        rec.imu_pkt_count = b.pkt_count;
        rec.imu_t_us = b.t_us;
        memcpy(rec.xl, b.sample.xl, sizeof(rec.xl));
        memcpy(rec.hg, b.sample.hg, sizeof(rec.hg));
        memcpy(rec.gy, b.sample.gy, sizeof(rec.gy));
        rec.imu_temp = b.sample.temp;
        rec.imu_flags = b.sample.flags;
        rec.imu_link_ok = (rec.t_us - b.sent_us) < 100000 ? 1 : 0;
        rec.baro_pressure = sd.baro_pressure;
        rec.baro_temp = sd.baro_temp;
        rec.mag_x = sd.mag_x; rec.mag_y = sd.mag_y; rec.mag_z = sd.mag_z;
        rec.mag_temp = sd.mag_temp;
        rec.sensor_t_ms = (uint32_t)(sd.timestamp_us / 1000);
        // Launch detection, from the same low-g accelerometer already in hand.
        if (!s_launched) {
            if (accel_magnitude_mg(&rec) >= TB_LOG_LAUNCH_MG) {
                if (++s_launch_run >= TB_LOG_LAUNCH_SAMPLES) {
                    s_launched = true;
                }
            } else {
                s_launch_run = 0;
            }
        }
        rec.launched = s_launched ? 1 : 0;
        rec.crc32 = esp_rom_crc32_le(0, (const uint8_t *)&rec, sizeof(rec) - sizeof(uint32_t));

        s_records++;
        if (s_sd && xQueueSend(s_sd_queue, &rec, 0) != pdTRUE) s_sd_drops++;
        if (s_launched && s_ring_count) nor_drain_ring();   // this record's own copy follows
        nor_offer(&rec);
    }
}

esp_err_t tb_log_start(FILE *sd_file) {
    s_sd = TB_LOG_SD_LIVE ? sd_file : NULL;
    if (s_sd) {
        setvbuf(s_sd, NULL, _IOFBF, SD_FILE_BUFFER);
        // One sync now, so the file exists on the card with a valid directory
        // entry before any flight data is written. Nothing syncs after this.
        fflush(s_sd);
        fsync(fileno(s_sd));
        s_sd_queue = xQueueCreate(SD_QUEUE_DEPTH, sizeof(tb_log_record_t));
        if (!s_sd_queue) return ESP_ERR_NO_MEM;
        if (xTaskCreatePinnedToCore(sd_writer_task, "sd_writer", 4096, NULL, 4, NULL, 0) != pdPASS) return ESP_FAIL;
    }
    if (xTaskCreatePinnedToCore(log_task, "tb_log", 6144, NULL, 5, NULL, 0) != pdPASS) return ESP_FAIL;
    ESP_LOGI(TAG, "logging %u-byte records at %d Hz to %s%s%s; NOR holds the last %d ms on the pad and starts at launch (|a| >= %d mg)", (unsigned)sizeof(tb_log_record_t), 1000 / TB_LOG_PERIOD_MS,
             nor_log_ok() ? "NOR" : "(no NOR)", s_sd ? " + SD (live)" : "",
             (!TB_LOG_SD_LIVE && sd_file == NULL) ? "" : (TB_LOG_SD_LIVE ? "" : "; SD gets the NOR dump at next boot"),
             TB_LOG_PRELAUNCH_MS, TB_LOG_LAUNCH_MG);
    return ESP_OK;
}

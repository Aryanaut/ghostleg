#ifndef TB_LOG_H
#define TB_LOG_H
// ---------------------------------------------------------------------------
// Training Board datalogger: one fixed-size record per TB_LOG_PERIOD_MS with
// the latest IMU sample (as streamed to Avionics C), barometer and
// magnetometer, written to both the SD card session file and the NOR log.
//
// Time sync with Avionics C: every record carries imu_t_us / imu_pkt_count of
// the most recent IMU frame sent over UART. Avionics C stores the same
// imu_t_us (plus its own receive time) in its telemetry records, so the two
// logs can be aligned even if the UART link dropped later.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <esp_err.h>
#include <stdbool.h>

// Live SD writes share SPI2 with the IMU. Measured 2026-09-03 with a SanDisk
// 64 GB card: single-sector writes and FATFS fsync held the bus for up to
// 695 ms, collapsing the IMU stream to ~30 Hz. So by default records go to
// the NOR flash only (one page program holds the bus < 1 ms) and the SD card
// receives the complete session as nor_dump_NNNNNN.bin at the next boot,
// before the bridge starts. Set to 1 to also write session_NNNNNN.bin live.
#define TB_LOG_SD_LIVE    1

// ---- Launch gating -------------------------------------------------------
// The SD card is 64 GB and logs continuously. The NOR flash is 16 MB, so it
// must not spend its capacity on pad time: records are held in a RAM ring
// while on the pad, and the ring is drained into the NOR the moment launch is
// detected. The result is a NOR log that begins TB_LOG_PRELAUNCH_MS before
// liftoff and never stops after it.
// Sized to comfortably exceed the worst measured fsync (334 ms), because a
// pad-time sync holds the SPI bus and so delays launch detection by up to that
// long - the ring has to still contain the launch when detection catches up.
#define TB_LOG_PRELAUNCH_MS   1000
#define TB_LOG_LAUNCH_MG      3000   // |low-g accel| over this counts toward launch
#define TB_LOG_LAUNCH_SAMPLES 3      // consecutive records above it before we commit

// fsync writes the FAT metadata: several random sector writes, measured at
// 296-334 ms with the IMU sharing the SPI bus. That is never worth paying once
// the vehicle is off the pad, so it is not paid after launch.
//
// On the pad it IS paid, once a second. Not doing so was a mistake: FAT only
// records a file's length in its directory entry when the file is synced or
// closed, so a session that ends with the power being pulled - which is every
// session on this board - left a file the host reads as ZERO BYTES. Measured on
// the card 2026-09-03: all four most recent sessions, 0 bytes. Combined with
// the launch-gated NOR holding only the pre-launch ring, a run that never
// triggered the launch gate recorded nothing recoverable at all.
//
// The original reasoning was that a third of a second on the pad is long enough
// to miss a launch. It is not, and the FIFO is why: the IMU is read through its
// 4 KB hardware FIFO (~810 ms at 240 Hz), so holding the bus DELAYS samples
// rather than destroying them - that is the whole point of reading it that way.
// Measured with pad syncs running, 2026-09-03: FIFO overruns 0, worst drain 39
// samples, and Avionics C counted zero sequence gaps across the whole run. The
// sync does not starve the IMU.
//
// It is slower than the old note claimed, though: worst fsync WALL time on this
// card is 992 ms, not 334. That is not one continuous bus hold - the bridge
// takes the bus back between the FAT's sector writes, and the worst continuous
// hold measured is 204 ms - but at one sync a second it was still enough to
// pull the record rate down to 68 Hz in the second it lands in.
//
// So the pad sync runs every TB_LOG_FSYNC_PERIOD_MS, not every second. The
// point of it is only that the file has a length if the power is pulled, and
// losing the last ten seconds of PAD data costs nothing, so paying a tenth as
// much bus time for it is strictly better. Launch detection is delayed by at
// most one continuous hold, and TB_LOG_PRELAUNCH_MS is five times that.
//
// After launch the cost is explicit and unchanged: an abrupt power loss
// truncates the live SD file to the moment of liftoff. The NOR flash is the
// guarantee for the flight itself - it holds everything from
// TB_LOG_PRELAUNCH_MS before liftoff onward, and the next boot copies it to the
// card. Power the board down cleanly (or reset it) to close the live file.
#define TB_LOG_FSYNC_IN_FLIGHT  0
#define TB_LOG_FSYNC_ON_PAD     1
#define TB_LOG_FSYNC_PERIOD_MS  10000

#define TB_LOG_MAGIC      0x42544253u   // "SBTB" little-endian
#define TB_LOG_VERSION    1
#define TB_LOG_PERIOD_MS  10            // 100 Hz records

typedef struct __attribute__((packed)) {
    uint32_t magic;          // TB_LOG_MAGIC
    uint16_t len;            // sizeof(tb_log_record_t)
    uint16_t version;        // TB_LOG_VERSION
    uint64_t t_us;           // training board esp_timer time when the record was built
    uint32_t imu_pkt_count;  // IMU frames sent over UART so far (low byte = wire seq)
    uint32_t imu_t_us;       // t_us field of the last IMU frame sent (32-bit, wraps ~71 min)
    int16_t  xl[3];          // raw low-g accel  (+-16 g full scale)
    int16_t  hg[3];          // raw high-g accel (+-64 g full scale)
    int16_t  gy[3];          // raw gyro         (+-2000 dps full scale)
    int16_t  imu_temp;       // raw IMU temperature
    uint8_t  imu_flags;      // IMU_LINK_FLAG_* of that frame
    uint8_t  imu_link_ok;    // 1 if an IMU frame was sent in the last 100 ms
    uint8_t  launched;       // 1 once launch has been detected (latches)
    uint8_t  _pad;
    float    baro_pressure;  // hPa
    float    baro_temp;      // degC
    float    mag_x, mag_y, mag_z;   // uT
    float    mag_temp;       // degC
    uint32_t sensor_t_ms;    // ms since boot of the last baro/mag update
    uint32_t crc32;          // CRC32 (esp_rom_crc32_le) of all preceding bytes
} tb_log_record_t;

// Start the sensor-poll/record task (and, if TB_LOG_SD_LIVE, the SD writer
// task). `sd_file` may be NULL; NOR writes go through nor_log_submit().
esp_err_t tb_log_start(FILE *sd_file);

// Diagnostics
uint32_t tb_log_records(void);
uint32_t tb_log_sd_drops(void);
uint32_t tb_log_sd_write_errors(void);
uint32_t tb_log_sd_max_write_us(void);   // longest fwrite/fsync since last call (resets)
uint32_t tb_log_sd_flush_max_us(void);   // longest fwrite alone (resets)
uint32_t tb_log_sd_sync_max_us(void);    // longest fsync alone (resets)
uint32_t tb_log_sd_block_writes(void);   // fwrites that took over 1 ms, i.e. actually hit the card
bool     tb_log_launched(void);          // launch detected (latched)
uint32_t tb_log_nor_buffered(void);      // records held in the pre-launch ring

#endif // TB_LOG_H

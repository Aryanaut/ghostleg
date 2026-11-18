#ifndef NOR_LOG_H
#define NOR_LOG_H
// ---------------------------------------------------------------------------
// Raw append log on an external SPI NOR flash (scheme ported from the
// Avionics B coprocessor's nor_writer, with a few hardening additions).
//
//  * Records are appended back-to-back from NOR_LOG_DATA_OFFSET; the write
//    pointer lives in NVS (namespace "storage", key "nor_pos") and is also
//    recovered by scanning forward for the first blank page, so a reset
//    between NVS commits never rewinds over live data.
//  * Every boot, the caller dumps [data_start, pos) verbatim to the SD card
//    (nor_log_dump_to_file) and ONLY after a verified dump wipes the region
//    (nor_log_wipe). No SD card / failed dump => the NOR keeps its data and
//    keeps appending; the next boot with a good card dumps the backlog.
//  * The wipe erases exactly the used region at boot, so during a session no
//    sector erase is ever needed (an erase holds the SPI bus for 45-400 ms,
//    which matters on the Training Board where the IMU shares the bus).
//    Just-in-time erase remains as a fallback if a wipe was interrupted.
//  * A writer task drains a queue; producers call nor_log_submit() which
//    never blocks (full queue => record dropped, counter incremented).
//
// Shared byte-identical between the Avionics C and Training Board repos.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_err.h>
#include <hal/spi_types.h>

#define NOR_LOG_MAX_RECORD   256          // largest record accepted by nor_log_submit()
#define NOR_LOG_QUEUE_DEPTH  256          // records buffered ahead of the writer task
#define NOR_LOG_DATA_OFFSET  4096u        // sector 0 reserved (unused header)

typedef struct {
    spi_host_device_t host;      // SPI bus (already initialised by the caller)
    int   cs_gpio;
    int   cs_id;                 // CS slot on the host (0..2)
    int   wp_gpio;               // -1 if not wired; driven HIGH
    int   hold_gpio;             // -1 if not wired; driven HIGH
    int   freq_mhz;
} nor_log_config_t;

// NVS must already be initialised (storage_nvs_init()). Probes the chip,
// recovers the write pointer, and performs a one-time full-chip erase the first
// time this firmware format meets a chip (takes ~30-60 s).
esp_err_t nor_log_init(const nor_log_config_t *cfg);
bool      nor_log_ok(void);
uint32_t  nor_log_chip_size(void);
uint32_t  nor_log_bytes_pending(void);   // bytes stored since the last wipe

// Copy [data_start, pos) to `path`, fsync, and verify the file size.
esp_err_t nor_log_dump_to_file(const char *path);
// Reset the write pointer and erase the used region. Call ONLY after a
// verified dump, and before nor_log_task_start().
esp_err_t nor_log_wipe(void);

// Writer task + non-blocking producer API.
esp_err_t nor_log_task_start(int priority, int core);
esp_err_t nor_log_submit(const void *record, size_t len);   // never blocks
void      nor_log_commit_position(void);                    // persist pos to NVS now

// Diagnostics
uint32_t nor_log_records_written(void);
uint32_t nor_log_drops(void);
uint32_t nor_log_write_errors(void);
uint32_t nor_log_max_write_us(void);     // longest single esp_flash_write since last call (resets)

#endif // NOR_LOG_H

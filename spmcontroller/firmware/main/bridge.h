#ifndef BRIDGE_H
#define BRIDGE_H
// IMU -> UART bridge task (the flight-critical low-latency path).
#include "sim_replay.h"
#include <stdint.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "imu.h"

typedef struct {
    imu_sample_t sample;    // last sample sent
    uint32_t     pkt_count; // frames sent so far (low byte = wire seq)
    uint32_t     t_us;      // t_us field of that frame
    int64_t      sent_us;   // esp_timer time when it was handed to the UART
} bridge_snapshot_t;

typedef struct {
    uint32_t packets_sent, interrupts, poll_fallbacks, empty_reads, spi_errors, uart_errors, partial_samples;
    uint32_t reads_over_1ms, reads_over_5ms;   // notify -> frames in UART took longer than this (SPI bus contention)
    uint32_t fifo_overruns;                    // FIFO wrapped: samples were genuinely lost
    uint32_t max_drain;                        // most samples recovered in one drain (1 = never fell behind)
    int64_t  read_time_sum_us, read_time_max_us;
} bridge_stats_t;

esp_err_t bridge_start(void);

// Register a task to be notified immediately after each IMU frame is sent.
// That is the start of the ~4 ms window before the next sample is due, which
// is when anything else sharing SPI2 should do its work. Pass NULL to stop.
void bridge_notify_after_tick(TaskHandle_t task);
void      bridge_get_snapshot(bridge_snapshot_t *out);
// Copies the counters and resets the per-interval ones (interrupts, poll_fallbacks, empty_reads, read_time_*, reads_over_*).
void      bridge_get_stats(bridge_stats_t *out);

#if SIM_REPLAY
// Hardware-in-the-loop clock, driven by real IMU sample delivery. See
// sim_replay.h. The barometer driver follows these rather than keeping its own
// clock, so the two sensors cannot drift apart within a board.
uint32_t bridge_sim_time_us(void);
float    bridge_sim_baro_alt_m(void);
#endif

#endif // BRIDGE_H

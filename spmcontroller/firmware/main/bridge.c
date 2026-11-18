// IMU -> UART bridge: INT1 data-ready -> ISR -> task notify -> SPI burst read
// -> 32-byte frame straight into the UART0 TX FIFO. Runs on core 1 at the
// highest priority so storage work on core 0 can only delay it through the
// shared SPI2 bus lock (measured and reported in the stats).
#include "bridge.h"
#include "sim_replay.h"

#if SIM_REPLAY
// Sim time, advanced one IMU sample period per sample actually read, and the
// barometric altitude the same clock says we are at. The barometer driver reads
// the latter rather than keeping its own clock, so the two sensors can never
// drift apart within a board.
static uint32_t s_sim_t_us = 0;
static float    s_sim_baro_alt_m = 0.0f;
uint32_t bridge_sim_time_us(void)   { return s_sim_t_us; }
float    bridge_sim_baro_alt_m(void) { return s_sim_baro_alt_m; }
#endif
#include "pins.h"
#include "imu_link.h"
#include "usb_stream.h"   // SPM rig: same samples, raw, to the PC over USB

#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#define TAG "bridge"
#define BRIDGE_TASK_PRIORITY   (configMAX_PRIORITIES - 2)
#define BRIDGE_TASK_CORE       1
#define IMU_POLL_FALLBACK_MS   10   // > one 240 Hz tick, so the interrupt normally wins

static TaskHandle_t s_task = NULL;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bridge_snapshot_t s_snap;
static bridge_stats_t s_stats;
static TaskHandle_t s_tick_task = NULL;
static uint32_t s_sample_t_us = 0;
static bool s_clock_valid = false;

static void IRAM_ATTR imu_int1_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static esp_err_t link_uart_init(void) {
    esp_err_t ret;
    uart_config_t cfg = {
        .baud_rate = IMU_LINK_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    // A TX ring buffer, so draining a FIFO backlog of many samples at once
    // queues rather than blocking this task frame by frame.
    if ((ret = uart_driver_install(IMU_LINK_UART_PORT, 256, 2048, 0, NULL, 0)) != ESP_OK) return ret;
    if ((ret = uart_param_config(IMU_LINK_UART_PORT, &cfg)) != ESP_OK) return ret;
    if ((ret = uart_set_pin(IMU_LINK_UART_PORT, IMU_LINK_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) return ret;
    ESP_LOGI(TAG, "IMU link UART%d TX=GPIO%d @ %d baud, frame %d bytes", IMU_LINK_UART_PORT, IMU_LINK_TX_PIN, IMU_LINK_BAUD, IMU_LINK_PACKET_LEN);
    return ESP_OK;
}

static void bridge_task(void *pvParameters) {
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << IMU_INT1_PIN), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_INT1_PIN, imu_int1_isr, NULL));
    ESP_LOGI(TAG, "bridge task running on core %d, INT1 on GPIO%d", xPortGetCoreID(), IMU_INT1_PIN);

    imu_link_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint32_t count = 0;
    static imu_sample_t batch[IMU_FIFO_MAX_SAMPLES];

    while (1) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_POLL_FALLBACK_MS));
        int64_t t0 = esp_timer_get_time();
        if (notified) s_stats.interrupts++; else s_stats.poll_fallbacks++;

        // Drain whatever the sensor has buffered. Normally one sample; after a
        // bus stall, everything that accumulated during it.
        bool overrun = false;
        int n = imu_read_fifo(batch, IMU_FIFO_MAX_SAMPLES, &overrun);
        if (n < 0) { s_stats.spi_errors++; continue; }
        if (overrun) s_stats.fifo_overruns++;
        if (n == 0) { s_stats.empty_reads++; continue; }
        if ((uint32_t)n > s_stats.max_drain) s_stats.max_drain = (uint32_t)n;

        // Timestamps come from a free-running sample clock that advances one
        // period per sample, NOT from back-dating the read time. The sensor
        // samples at a fixed rate, so this is both the true spacing and
        // strictly monotonic. Back-dating from the read time is neither: when
        // a drain returns more samples than the time since the last drain
        // accounts for, the oldest of them get stamped BEFORE samples already
        // sent, which is what the receiver saw as negative intervals.
        const uint32_t period_us = 1000000u / IMU_LINK_RATE_HZ;
        if (!s_clock_valid) {
            s_sample_t_us = (uint32_t)t0 - (uint32_t)((n - 1) * period_us);
            s_clock_valid = true;
        }
        // Re-anchor only when the sensor genuinely lost samples (FIFO wrapped)
        // or the clock has fallen implausibly far behind, and only forwards,
        // so the sequence can never step back.
        int32_t behind = (int32_t)((uint32_t)t0 - s_sample_t_us);
        if (overrun || behind > (int32_t)(period_us * IMU_LINK_RATE_HZ / 4)) {   // 250 ms
            s_sample_t_us = (uint32_t)t0 - (uint32_t)((n - 1) * period_us);
        }

#if SIM_REPLAY
        // Hardware-in-the-loop. The FIFO read above was real - real interrupt,
        // real SPI transaction, real drain count after a real stall - and only
        // the numbers in it are replaced. Sim time advances one sample period
        // per sample actually read, so it follows real delivery rather than
        // wall clock, and it goes out on the wire in t_us where Avionics C can
        // use it to index its own barometer replay.
        for (int i = 0; i < n; i++) {
            float acc_mg[3], gyr_dps[3], alt_m;
            sim_replay_at(s_sim_t_us + (uint32_t)i * period_us, acc_mg, gyr_dps, &alt_m);
            for (int ax = 0; ax < 3; ax++) {
                // Through the sensors' own full scales, so the +-16 g low-g
                // rail and the +-64 g high-g rail clip exactly as the part does.
                float lg = acc_mg[ax] / 0.488f, hg = acc_mg[ax] / 1.952f;
                float gy = gyr_dps[ax] * 1000.0f / 70.0f;
                batch[i].xl[ax] = (int16_t)(lg >  32767.0f ?  32767.0f : lg < -32768.0f ? -32768.0f : lg);
                batch[i].hg[ax] = (int16_t)(hg >  32767.0f ?  32767.0f : hg < -32768.0f ? -32768.0f : hg);
                batch[i].gy[ax] = (int16_t)(gy >  32767.0f ?  32767.0f : gy < -32768.0f ? -32768.0f : gy);
            }
            batch[i].flags |= IMU_LINK_FLAG_XL | IMU_LINK_FLAG_HG | IMU_LINK_FLAG_GY;
            s_sim_baro_alt_m = alt_m;   // the barometer replay follows the same clock
        }
#endif

        for (int i = 0; i < n; i++) {
            const imu_sample_t *s = &batch[i];
            pkt.seq  = (uint8_t)count;
#if SIM_REPLAY
            pkt.t_us = s_sim_t_us;
            s_sim_t_us += period_us;
#else
            pkt.t_us = s_sample_t_us;
#endif
            s_sample_t_us += period_us;
            memcpy(pkt.xl, s->xl, sizeof(pkt.xl));
            memcpy(pkt.hg, s->hg, sizeof(pkt.hg));
            memcpy(pkt.gy, s->gy, sizeof(pkt.gy));
            pkt.flags = s->flags;   // temperature is logged locally, never sent
            imu_link_finalize(&pkt);

            int written = uart_write_bytes(IMU_LINK_UART_PORT, &pkt, sizeof(pkt));
            // Same sample, same seq/timestamp, to the PC over USB (non-blocking).
            usb_stream_send_imu(count, pkt.t_us, s);
            count++;
            if (written != (int)sizeof(pkt)) s_stats.uart_errors++; else s_stats.packets_sent++;
            if ((s->flags & (IMU_LINK_FLAG_XL | IMU_LINK_FLAG_HG | IMU_LINK_FLAG_GY)) !=
                (IMU_LINK_FLAG_XL | IMU_LINK_FLAG_HG | IMU_LINK_FLAG_GY)) s_stats.partial_samples++;
        }

        // Hand the SPI bus window to the storage writer now that the sensor is drained.
        TaskHandle_t t = s_tick_task;
        if (t) xTaskNotifyGive(t);

        int64_t t1 = esp_timer_get_time();
        int64_t dt = t1 - t0;
        s_stats.read_time_sum_us += dt;
        if (dt > s_stats.read_time_max_us) s_stats.read_time_max_us = dt;
        if (dt > 1000) s_stats.reads_over_1ms++;
        if (dt > 5000) s_stats.reads_over_5ms++;

        portENTER_CRITICAL(&s_lock);
        s_snap.sample = batch[n - 1];
        s_snap.pkt_count = count;
        s_snap.t_us = pkt.t_us;
        s_snap.sent_us = t1;
        portEXIT_CRITICAL(&s_lock);
    }
}

esp_err_t bridge_start(void) {
    esp_err_t r = link_uart_init();
    if (r != ESP_OK) { ESP_LOGE(TAG, "link UART init failed: %s", esp_err_to_name(r)); return r; }
    if (xTaskCreatePinnedToCore(bridge_task, "imu_bridge", 4096, NULL, BRIDGE_TASK_PRIORITY, &s_task, BRIDGE_TASK_CORE) != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

void bridge_notify_after_tick(TaskHandle_t task) { s_tick_task = task; }

void bridge_get_snapshot(bridge_snapshot_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = s_snap;
    portEXIT_CRITICAL(&s_lock);
}

void bridge_get_stats(bridge_stats_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    s_stats.interrupts = s_stats.poll_fallbacks = s_stats.empty_reads = 0;
    s_stats.reads_over_1ms = s_stats.reads_over_5ms = 0;
    s_stats.read_time_sum_us = s_stats.read_time_max_us = 0;
    s_stats.max_drain = 0;
    portEXIT_CRITICAL(&s_lock);
}

// Raw sensor stream over USB Serial/JTAG - see usb_stream.h for the protocol.
//
// Data flow:
//   producers (IMU bridge task @240 Hz, mag in the log task, ESP_LOG anywhere)
//     -> build one complete frame on their own stack
//     -> xRingbufferSend(..., timeout 0)           atomic, never blocks the IMU
//   usb_writer task (core 0, low priority)
//     -> xRingbufferReceive -> usb_serial_jtag_write_bytes
// A NOSPLIT ring buffer stores each frame as one item, so frames are written
// whole and in order. When the PC is not draining the port the ring fills and
// new frames are dropped (counted in INFO), which is the right failure mode
// for a live stream: the IMU path keeps its timing.
#include "usb_stream.h"
#include "imu_link.h"          // imu_link_crc16()
#include "lsm6dsv80x_reg.h"    // LSB -> unit conversions (for INFO scale factors)

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <driver/usb_serial_jtag.h>

#define TAG "usb_stream"

#define RING_BYTES        (16 * 1024)   // ~0.5 s of IMU frames at 240 Hz
#define USB_TX_BUF_BYTES  4096          // driver's own TX buffer
#define WRITER_PRIORITY   4             // same as sd_writer, below tb_log (5); core 0, so never competes with the IMU bridge on core 1
#define WRITER_CORE       0
#define INFO_PERIOD_US    1000000
#define FRAME_OVERHEAD    6             // sync0 sync1 type len + crc16

static RingbufHandle_t s_ring = NULL;
static volatile uint32_t s_dropped = 0;
static volatile uint32_t s_imu_frames = 0;
static vprintf_like_t s_prev_vprintf = NULL;

// ---- frame building ---------------------------------------------------------

// Build a frame into `out` (which must hold len + FRAME_OVERHEAD bytes) and
// return its total length.
static size_t build_frame(uint8_t *out, uint8_t type, const void *payload, uint8_t len) {
    out[0] = USB_STREAM_SYNC0;
    out[1] = USB_STREAM_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len) memcpy(&out[4], payload, len);
    uint16_t crc = imu_link_crc16(&out[2], (size_t)len + 2);   // over type, len, payload
    out[4 + len] = (uint8_t)(crc & 0xFF);
    out[5 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + FRAME_OVERHEAD;
}

// Queue one frame without blocking. Returns false (and counts a drop) if the
// ring is full or the stream is not running.
static bool push_frame(uint8_t type, const void *payload, uint8_t len, TickType_t wait) {
    if (!s_ring) return false;
    uint8_t frame[USB_STREAM_MAX_PAYLOAD + FRAME_OVERHEAD];
    size_t n = build_frame(frame, type, payload, len);
    if (xRingbufferSend(s_ring, frame, n, wait) != pdTRUE) {
        s_dropped++;
        return false;
    }
    return true;
}

// ---- producers ---------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t t_us;
    int16_t  xl[3];
    int16_t  gy[3];
    int16_t  hg[3];
    uint8_t  flags;
} imu_payload_t;                       // 27 bytes
_Static_assert(sizeof(imu_payload_t) == 27, "IMU payload layout changed - bump USB_STREAM_PROTO");

void usb_stream_send_imu(uint32_t seq, uint32_t t_us, const imu_sample_t *s) {
    imu_payload_t p = { .seq = seq, .t_us = t_us, .flags = s->flags };
    memcpy(p.xl, s->xl, sizeof(p.xl));
    memcpy(p.gy, s->gy, sizeof(p.gy));
    memcpy(p.hg, s->hg, sizeof(p.hg));
    if (push_frame(USB_STREAM_T_IMU, &p, sizeof(p), 0)) s_imu_frames++;   // 0 = never block the bridge
}

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    float    mag[3];
    float    temp;
} mag_payload_t;                       // 20 bytes
_Static_assert(sizeof(mag_payload_t) == 20, "MAG payload layout changed - bump USB_STREAM_PROTO");

void usb_stream_send_mag(const float mag_ut[3], float temp_c) {
    mag_payload_t p = { .t_us = (uint32_t)esp_timer_get_time(), .temp = temp_c };
    memcpy(p.mag, mag_ut, sizeof(p.mag));
    push_frame(USB_STREAM_T_MAG, &p, sizeof(p), 0);
}

typedef struct __attribute__((packed)) {
    uint8_t  proto;
    uint16_t odr_hz;
    float    xl_mg_per_lsb;
    float    gy_mdps_per_lsb;
    float    hg_mg_per_lsb;
    uint32_t dropped;
    uint32_t imu_frames;
    uint32_t uptime_ms;
} info_payload_t;                      // 27 bytes
_Static_assert(sizeof(info_payload_t) == 27, "INFO payload layout changed - bump USB_STREAM_PROTO");

static void send_info(void) {
    info_payload_t p = {
        .proto = USB_STREAM_PROTO,
        .odr_hz = IMU_LINK_RATE_HZ,
        // The conversion macros in imu.h are linear, so converting 1 LSB gives
        // the sensitivity for whatever full scale this build was compiled with.
        .xl_mg_per_lsb = LG_ACCEL_TO_MG(1),
        .gy_mdps_per_lsb = GYRO_TO_MDPS(1),
        .hg_mg_per_lsb = HG_ACCEL_TO_MG(1),
        .dropped = s_dropped,
        .imu_frames = s_imu_frames,
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };
    push_frame(USB_STREAM_T_INFO, &p, sizeof(p), 0);
}

// ESP_LOG redirection: format the line, then ship it as one or more TEXT
// frames. Blocks for at most a few ms if the ring is full (log calls never
// come from the IMU bridge's sample loop).
static int usb_log_vprintf(const char *fmt, va_list ap) {
    char line[512];
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    if (n <= 0) return n;
    int len = n < (int)sizeof(line) ? n : (int)sizeof(line) - 1;
    for (int off = 0; off < len; off += USB_STREAM_MAX_PAYLOAD) {
        int chunk = len - off > USB_STREAM_MAX_PAYLOAD ? USB_STREAM_MAX_PAYLOAD : len - off;
        push_frame(USB_STREAM_T_TEXT, &line[off], (uint8_t)chunk, pdMS_TO_TICKS(5));
    }
    return n;
}

// ---- writer task ---------------------------------------------------------------

static void usb_writer_task(void *arg) {
    int64_t next_info = esp_timer_get_time() + INFO_PERIOD_US;
    while (1) {
        size_t len = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(s_ring, &len, pdMS_TO_TICKS(50));
        if (item) {
            // Only write while a host has the port open; otherwise the driver
            // would sit on its timeout and the ring would back up for nothing.
            if (usb_serial_jtag_is_connected()) {
                int w = usb_serial_jtag_write_bytes(item, len, pdMS_TO_TICKS(20));
                if (w != (int)len) s_dropped++;
            } else {
                s_dropped++;
            }
            vRingbufferReturnItem(s_ring, item);
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_info) {
            send_info();
            next_info = now + INFO_PERIOD_US;
        }
    }
}

esp_err_t usb_stream_init(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = USB_TX_BUF_BYTES,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) return err;

    s_ring = xRingbufferCreate(RING_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (!s_ring) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(usb_writer_task, "usb_writer", 3072, NULL,
                                WRITER_PRIORITY, NULL, WRITER_CORE) != pdPASS) {
        return ESP_FAIL;
    }
    // From here on every ESP_LOG line is framed. (Bootloader / panic output
    // still goes out raw; the host prints unframed bytes as text.)
    s_prev_vprintf = esp_log_set_vprintf(usb_log_vprintf);
    ESP_LOGI(TAG, "USB sensor stream v%d up: IMU %d Hz + mag + framed logs", USB_STREAM_PROTO, IMU_LINK_RATE_HZ);
    return ESP_OK;
}

uint32_t usb_stream_dropped(void) { return s_dropped; }

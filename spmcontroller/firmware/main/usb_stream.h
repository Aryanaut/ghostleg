#ifndef USB_STREAM_H
#define USB_STREAM_H
// ---------------------------------------------------------------------------
// Raw sensor stream to a PC over the ESP32-S3's native USB Serial/JTAG port
// (the same USB-C port used for flashing / logs). Added for the SPM hip-joint
// rig: the board is rigidly mounted on the femur standoff, and the laptop
// fuses these raw samples into the end-effector's orientation and angular
// velocity (spmcontroller/spm/imu_board.py).
//
// Everything that leaves the USB port after usb_stream_init() is framed -
// including ESP_LOG text, which is re-routed into TEXT frames - so log lines
// can never land in the middle of a binary sample.
//
// Frame (little-endian, no padding):
//   [0]      0xA5          sync 0
//   [1]      0x96          sync 1 (differs from the UART link's 0x58, so the
//                          two protocols can never be confused)
//   [2]      type          USB_STREAM_T_*
//   [3]      len           payload length, 0..USB_STREAM_MAX_PAYLOAD
//   [4..]    payload
//   [4+len]  crc16 (LE)    CRC-16/CCITT-FALSE over bytes [2 .. 3+len]
//                          (type, len and payload; same CRC as imu_link.c)
//
// Payloads:
//   IMU  (0x01, 27 B, one per IMU sample, 240 Hz):
//        u32 seq | u32 t_us | i16 xl[3] | i16 gy[3] | i16 hg[3] | u8 flags
//        raw LSB, sensor axes; t_us = the same free-running sample clock that
//        is sent on the UART link (one period per sample, monotonic).
//   MAG  (0x02, 20 B, ~100 Hz): u32 t_us | f32 mag_uT[3] | f32 temp_C
//   TEXT (0x03, n B): one chunk of ESP_LOG output (UTF-8, no terminator)
//   INFO (0x04, 27 B, 1 Hz):
//        u8 proto | u16 odr_hz | f32 xl_mg_per_lsb | f32 gy_mdps_per_lsb |
//        f32 hg_mg_per_lsb | u32 frames_dropped | u32 imu_frames | u32 uptime_ms
//        The host takes its scale factors from here, so changing the IMU full
//        scale in imu.h needs no host change.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <esp_err.h>
#include "imu.h"

#define USB_STREAM_PROTO        1
#define USB_STREAM_SYNC0        0xA5
#define USB_STREAM_SYNC1        0x96
#define USB_STREAM_MAX_PAYLOAD  240

#define USB_STREAM_T_IMU   0x01
#define USB_STREAM_T_MAG   0x02
#define USB_STREAM_T_TEXT  0x03
#define USB_STREAM_T_INFO  0x04

// Install the USB Serial/JTAG driver, start the writer task (core 0) and
// redirect ESP_LOG output into TEXT frames. Call first thing in app_main.
esp_err_t usb_stream_init(void);

// Non-blocking producers. Safe to call from any task (not from an ISR).
// If the PC is not reading, frames are dropped and counted - never blocking
// the IMU bridge task.
void usb_stream_send_imu(uint32_t seq, uint32_t t_us, const imu_sample_t *s);
void usb_stream_send_mag(const float mag_ut[3], float temp_c);

uint32_t usb_stream_dropped(void);

#endif // USB_STREAM_H

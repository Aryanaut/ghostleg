#ifndef IMU_LINK_H
#define IMU_LINK_H
// ---------------------------------------------------------------------------
// IMU-over-UART link protocol shared by the Training Board (sender) and
// Avionics C (receiver). Keep this file and imu_link.c byte-identical in both
// repositories. Pure C, no ESP-IDF dependencies, so it can be unit tested on
// the host (see tools/imu_link_test.c).
//
// Wiring (Training Board J4 -> Avionics C J4, JST-XH "UART breakout for backup GPS"):
//   TB J4.1  +3.3V  -> C J4.1  +3.3V
//   TB J4.2  GND    -> C J4.2  GND   (any GND)
//   TB J4.15 TX0    -> C J4.3  RX0   (GPIO43 on TB -> GPIO44 on C)
//
// Frame layout (29 bytes, little-endian, no padding):
//   [0]     sync0     0xA5
//   [1]     sync1     0x5A ^ IMU_LINK_VERSION  -- see "version" below
//   [2]     seq       increments every packet (gaps => dropped packets)
//   [3..6]  t_us      sender esp_timer time, microseconds (wraps every ~71 min)
//   [7..12] xl[3]     low-g  accel raw LSB  (+-16 g full scale)
//   [13..18] hg[3]    high-g accel raw LSB  (+-64 g full scale)
//   [19..24] gy[3]    gyro raw LSB          (+-2000 dps full scale)
//   [25]    flags     IMU_LINK_FLAG_* bits: which groups are fresh this packet
//   [26]    checksum  8-bit sum of bytes [2..25]
//   [27..28] crc16    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of bytes [0..26], LE
//
// No temperature: Avionics C does no temperature compensation. The Training
// Board still reads it (it comes free in the same burst) and logs it locally.
//
// The protocol version is folded INTO the sync word rather than carried as its
// own byte. It costs nothing on the wire and gives the same protection: a
// receiver built for a different version simply never finds a frame header,
// instead of silently decoding shifted fields that still pass CRC. Bump
// IMU_LINK_VERSION for ANY change to the layout below.
//
// Every packet is validated with BOTH the checksum (cheap early reject) and
// the CRC before it is accepted. Corrupt bytes cause a resync scan for the
// next plausible header, so a single bad byte only costs one packet.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define IMU_LINK_BAUD        921600
#define IMU_LINK_VERSION     2
#define IMU_LINK_SYNC0       0xA5
#define IMU_LINK_SYNC1       (0x5A ^ IMU_LINK_VERSION)

// Frames per second on the link. The IMU is configured to the nearest output
// rate at or above this (240 Hz); 200 Hz itself is not one of the part's
// available rates. One frame per sample, so this is also the sample rate.
#define IMU_LINK_RATE_HZ     240

#define IMU_LINK_FLAG_XL     (1u << 0)  // low-g accel fresh
#define IMU_LINK_FLAG_HG     (1u << 1)  // high-g accel fresh
#define IMU_LINK_FLAG_GY     (1u << 2)  // gyro fresh
#define IMU_LINK_FLAG_TEMP   (1u << 3)  // temperature fresh (Training Board local logging only, never sent)

typedef struct __attribute__((packed)) {
    uint8_t  sync0;
    uint8_t  sync1;
    uint8_t  seq;
    uint32_t t_us;
    int16_t  xl[3];
    int16_t  hg[3];
    int16_t  gy[3];
    uint8_t  flags;
    uint8_t  checksum;
    uint8_t  crc[2];   // little-endian CRC-16
} imu_link_packet_t;

#define IMU_LINK_PACKET_LEN  ((int)sizeof(imu_link_packet_t))   // 29

// --- sender side ------------------------------------------------------------
uint16_t imu_link_crc16(const uint8_t *data, size_t len);
uint8_t  imu_link_checksum(const uint8_t *data, size_t len);
// Fill in sync bytes, checksum and CRC. Caller sets seq/t_us/data/flags.
void     imu_link_finalize(imu_link_packet_t *pkt);

// --- receiver side ----------------------------------------------------------
typedef struct {
    uint8_t  buf[sizeof(imu_link_packet_t)];
    uint8_t  pos;
    bool     have_seq;
    uint8_t  last_seq;
    // statistics (monotonic counters)
    uint32_t packets_ok;
    uint32_t checksum_err;
    uint32_t crc_err;
    uint32_t version_err;   // frames whose sync word did not match this build's version
    uint32_t resyncs;        // times the parser had to hunt for a new header
    uint32_t bytes_dropped;  // bytes discarded while hunting for sync
    uint32_t seq_gaps;       // packets missing according to seq numbers
} imu_link_parser_t;

void imu_link_parser_init(imu_link_parser_t *p);
// Feed one received byte. Returns true when a fully validated packet is
// available in *out (out may be NULL if only stats are wanted).
bool imu_link_parser_feed(imu_link_parser_t *p, uint8_t byte, imu_link_packet_t *out);

#endif // IMU_LINK_H

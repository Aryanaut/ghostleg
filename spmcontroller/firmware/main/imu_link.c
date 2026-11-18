#include "imu_link.h"
#include <string.h>

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
uint16_t imu_link_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint8_t imu_link_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum = (uint8_t)(sum + data[i]);
    return sum;
}

#define CHK_START   offsetof(imu_link_packet_t, seq)
#define CHK_END     offsetof(imu_link_packet_t, checksum)          // exclusive
#define CRC_END     offsetof(imu_link_packet_t, crc)               // exclusive

void imu_link_finalize(imu_link_packet_t *pkt) {
    uint8_t *b = (uint8_t *)pkt;
    pkt->sync0   = IMU_LINK_SYNC0;
    pkt->sync1   = IMU_LINK_SYNC1;
    pkt->checksum = imu_link_checksum(b + CHK_START, CHK_END - CHK_START);
    uint16_t crc = imu_link_crc16(b, CRC_END);
    pkt->crc[0] = (uint8_t)(crc & 0xFF);
    pkt->crc[1] = (uint8_t)(crc >> 8);
}

void imu_link_parser_init(imu_link_parser_t *p) {
    memset(p, 0, sizeof(*p));
}

// Header plausibility check for a candidate start index inside buf[0..n).
// Only the bytes that have actually arrived are compared.
static bool header_plausible(const uint8_t *buf, size_t n, size_t i) {
    if (buf[i] != IMU_LINK_SYNC0) return false;
    if (i + 1 < n && buf[i + 1] != IMU_LINK_SYNC1) return false;
    return true;
}

// Discard buf[0] and shift the buffer to the next plausible header start.
static void resync(imu_link_parser_t *p) {
    size_t n = p->pos;
    size_t i = 1;
    while (i < n && !header_plausible(p->buf, n, i)) i++;
    if (i < n) {
        memmove(p->buf, p->buf + i, n - i);
        p->pos = (uint8_t)(n - i);
    } else {
        p->pos = 0;
    }
    p->bytes_dropped += (uint32_t)i;
    p->resyncs++;
}

bool imu_link_parser_feed(imu_link_parser_t *p, uint8_t byte, imu_link_packet_t *out) {
    if (p->pos == 0) {
        if (byte != IMU_LINK_SYNC0) { p->bytes_dropped++; return false; }
        p->buf[p->pos++] = byte;
        return false;
    }

    p->buf[p->pos++] = byte;

    // Early header validation so garbage is rejected as soon as possible.
    // sync1 carries the protocol version, so a mismatched build never syncs.
    if (p->pos == 2 && byte != IMU_LINK_SYNC1) {
        if (byte == 0x5A) p->version_err++;   // a frame from a different protocol version
        resync(p);
        return false;
    }

    if (p->pos < sizeof(imu_link_packet_t)) return false;

    // Full frame collected: validate checksum then CRC.
    const uint8_t *b = p->buf;
    uint8_t chk = imu_link_checksum(b + CHK_START, CHK_END - CHK_START);
    if (chk != b[CHK_END]) { p->checksum_err++; resync(p); return false; }

    uint16_t crc = imu_link_crc16(b, CRC_END);
    uint16_t rx_crc = (uint16_t)b[CRC_END] | ((uint16_t)b[CRC_END + 1] << 8);
    if (crc != rx_crc) { p->crc_err++; resync(p); return false; }

    // Valid packet.
    imu_link_packet_t pkt;
    memcpy(&pkt, b, sizeof(pkt));
    if (p->have_seq) {
        uint8_t expected = (uint8_t)(p->last_seq + 1);
        uint8_t gap = (uint8_t)(pkt.seq - expected);
        p->seq_gaps += gap;
    }
    p->have_seq = true;
    p->last_seq = pkt.seq;
    p->packets_ok++;
    p->pos = 0;
    if (out) *out = pkt;
    return true;
}

#include "sim_replay.h"

#if SIM_REPLAY

#include "sim_flight.h"
#include <math.h>
#include <stdio.h>

// Binary search rather than a cursor: both boards call this from more than one
// task, and 4000 rows is twelve comparisons, which is nothing next to the SPI
// transaction it sits inside.
static int row_before(uint32_t t_ms) {
    int lo = 0, hi = SIM_FLIGHT_ROWS - 1;
    if (t_ms <= SIM_FLIGHT[0].t_ms) return 0;
    if (t_ms >= SIM_FLIGHT[hi].t_ms) return hi;
    while (lo + 1 < hi) {
        int mid = (lo + hi) / 2;
        if (SIM_FLIGHT[mid].t_ms <= t_ms) lo = mid; else hi = mid;
    }
    return lo;
}

void sim_replay_at(uint32_t t_us, float accel_mg[3], float gyro_dps[3], float *alt_m) {
    uint32_t t_ms = t_us / 1000u;
    int i = row_before(t_ms);
    int j = (i + 1 < SIM_FLIGHT_ROWS) ? i + 1 : i;
    const sim_flight_row_t *a = &SIM_FLIGHT[i], *b = &SIM_FLIGHT[j];

    float f = 0.0f;
    if (b->t_ms > a->t_ms) {
        f = (float)(t_ms - a->t_ms) / (float)(b->t_ms - a->t_ms);
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
    }
    #define LERP(x, y) ((float)(x) + f * ((float)(y) - (float)(x)))

    // Table order is the trajectory's (accX, accY, axial); map it onto the IMU.
    float ax = LERP(a->acc_10mg[0], b->acc_10mg[0]) * 10.0f;
    float ay = LERP(a->acc_10mg[1], b->acc_10mg[1]) * 10.0f;
    float az = LERP(a->acc_10mg[2], b->acc_10mg[2]) * 10.0f;
    accel_mg[SIM_AXIS_LATERAL_A] = ax;
    accel_mg[SIM_AXIS_LATERAL_B] = ay;
    accel_mg[SIM_AXIS_AXIAL]     = az;

    float gx = LERP(a->gyr_10cdps[0], b->gyr_10cdps[0]) * 0.1f;
    float gy = LERP(a->gyr_10cdps[1], b->gyr_10cdps[1]) * 0.1f;
    float gz = LERP(a->gyr_10cdps[2], b->gyr_10cdps[2]) * 0.1f;
    gyro_dps[SIM_AXIS_LATERAL_A] = gx;
    gyro_dps[SIM_AXIS_LATERAL_B] = gy;
    gyro_dps[SIM_AXIS_AXIAL]     = gz;

    *alt_m = LERP(a->alt_cm, b->alt_cm) * 0.01f;
    #undef LERP
}

float sim_replay_pressure_hpa(float alt_above_pad_m) {
    // The inverse of the barometric formula flight_state.c uses, so that what
    // the estimator recovers is the altitude the table asked for.
    float h = alt_above_pad_m + SIM_PAD_ALTITUDE_M;
    return 1013.25f * powf(1.0f - h / 44330.0f, 1.0f / 0.190295f);
}

bool sim_replay_finished(uint32_t t_us) {
    return (t_us / 1000u) > SIM_FLIGHT[SIM_FLIGHT_ROWS - 1].t_ms;
}

const char *sim_replay_describe(void) {
    static char buf[160];
    snprintf(buf, sizeof(buf),
             "SIM REPLAY: %s, apogee %.0f m, peak Mach %.2f, %d rows, ignition at t=%.1f s, ends t=%.1f s",
             SIM_FLIGHT_SOURCE, (double)SIM_FLIGHT_APOGEE_M, (double)SIM_FLIGHT_PEAK_MACH,
             SIM_FLIGHT_ROWS, SIM_FLIGHT_IGNITION_MS / 1000.0, SIM_FLIGHT_DURATION_MS / 1000.0);
    return buf;
}

#endif // SIM_REPLAY

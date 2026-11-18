"""
imu_check.py - smoke test of the femur IMU board's USB stream (default 8 s).
Writes imu_check_log.txt next to this file.
    python experiments/imu_check.py              # 8 s
    python experiments/imu_check.py --seconds 30 # longer = tighter rate estimate
(Testing/characterisation script - kept out of the main app on purpose.)

Sample-rate check (HANDOFF 6.1): the host's SampleClock measures the sensor's
true rate from PC arrival times.  As an INDEPENDENT cross-check this script
also fits the board's own clock: each INFO frame carries esp_timer uptime_ms
and follows (in the same ordered USB stream) the last IMU frame queued before
it, so  (seq of the last IMU frame before INFO, uptime_ms)  pairs give the
rate on the ESP32's crystal, with no PC/USB timing involved.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spm.imu_board import ImuBoard, find_board_port   # noqa: E402

OUT = Path(__file__).with_name("imu_check_log.txt")
lines = []
def log(s=""):
    print(s); lines.append(str(s))

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=8.0)
SECS = ap.parse_args().seconds

try:
    port = find_board_port()
    log(f"Espressif port found: {port}")
    board = ImuBoard(port)
    samples = []
    board.sample_listeners.append(lambda t, g, a: samples.append((t, *g, *a, board.t_board)))
    # cross-check data: (last IMU seq seen, board uptime ms) at every INFO frame
    info_pairs = []
    _on_imu = board._on_imu
    raw = bytearray()
    _feed = board.feed
    def feed_and_keep(data, t_rx=None):  # keep a raw copy for offline diagnosis
        raw.extend(data)
        n_before = board.stats.board_uptime_s
        _feed(data, t_rx)
        # an INFO in this chunk changed uptime; pair it with the last seq parsed.
        # (Approximate if IMU frames followed INFO in the same chunk; the
        # regression over many INFOs averages that out to < 1 sample.)
        if board.stats.board_uptime_s != n_before and board.seq is not None:
            info_pairs.append((board.seq, board.stats.board_uptime_s))
    board.feed = feed_and_keep
    board.open()
    t0 = time.monotonic()
    time.sleep(SECS)
    board.close()
    Path(__file__).with_name("imu_raw.bin").write_bytes(bytes(raw))
    s = board.snapshot()
    log(f"INFO frame received: {board.got_info}  odr={board.odr_hz} Hz  "
        f"accel {board.xl_g_per_lsb*1e3:.4f} mg/LSB  gyro {math.degrees(board.gy_rad_per_lsb)*1e3:.2f} mdps/LSB")
    log(f"IMU frames: {s['frames']} in {SECS:g} s  -> {s['frames']/SECS:.1f} Hz | seq gaps {s['seq_gaps']} | "
        f"CRC errors {s['crc_errors']} | board-side drops {s['board_dropped']} | uptime {s['uptime_s']:.1f} s")
    log(f"mag frames: {board.stats.mag_frames}  last mag [uT] = {s['mag_uT']}")
    if samples:
        A = np.array(samples)
        t = A[:, 0]; gy = np.degrees(A[:, 1:4]); ac = A[:, 4:7]; tb = A[:, 7]
        dtb = np.diff(tb); dt = np.diff(t)
        log(f"firmware t_us (nominal clock): dt mean {dtb.mean()*1e3:.3f} ms, min {dtb.min()*1e3:.3f}, "
            f"max {dtb.max()*1e3:.3f}  ({int((dtb > 0.1).sum())} re-anchor jumps)")
        log(f"host corrected time:           dt mean {dt.mean()*1e3:.3f} ms, min {dt.min()*1e3:.3f}, max {dt.max()*1e3:.3f}")
        log(f"TRUE sample rate (host SampleClock): {board.clock.rate_hz:.3f} Hz  converged={board.clock.converged}  "
            f"envelope rms {board.clock.fit_rms_ms:.3f} ms  -> nominal/true = {board.odr_hz / board.clock.rate_hz:.4f}")
        if len(info_pairs) >= 3:
            P = np.array(info_pairs, float)
            slope = np.polyfit(P[:, 1], P[:, 0], 1)[0]           # samples per second of board time
            log(f"cross-check (board esp_timer vs seq, {len(info_pairs)} INFO frames): {slope:.3f} Hz  "
                f"(differs from host estimate by {abs(slope - board.clock.rate_hz) / slope * 100:.3f}%)")
        log(f"accel mean [g] = {np.round(ac.mean(0), 4)}  |a| = {np.linalg.norm(ac.mean(0)):.4f} g  std {np.round(ac.std(0)*1e3, 2)} mg")
        log(f"gyro  mean [dps] = {np.round(gy.mean(0), 3)}  std {np.round(gy.std(0), 3)} dps")
        up = ac.mean(0) / np.linalg.norm(ac.mean(0))
        log(f"board 'up' axis (sensor frame) = {np.round(up, 3)}  -> tilt of sensor z from vertical {math.degrees(math.acos(np.clip(up[2], -1, 1))):.1f} deg")
    log("--- last board log lines ---")
    for ln in list(board.text_lines)[-12:]:
        log("  " + ln)
except Exception as exc:
    log(f"ERROR: {exc!r}")
OUT.write_text("\n".join(lines) + "\n")

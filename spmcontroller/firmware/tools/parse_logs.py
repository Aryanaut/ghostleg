#!/usr/bin/env python3
"""Parse and cross-check the binary logs written by Avionics C and the Space Bar
Training Board.

Both boards write the same bytes to two places: an SD card session file written
live, and the NOR flash, which is copied to the SD card as nor_dump_NNNNNN.bin at
the NEXT boot. So a card normally holds

    session_NNNNNN.bin    what the board wrote live during boot NNNNNN
    nor_dump_NNNNNN.bin   the NOR backlog dumped at the start of boot NNNNNN,
                          i.e. the record of the PREVIOUS session

Record formats are auto-detected from the magic word:

    "SPB2"  Avionics C TelemetryPacket, record format 2   (current)
    "SPBR"  Avionics C TelemetryPacket, record format 1   (older cards)
    "SBTB"  Training Board tb_log_record (IMU as sent over UART + baro + mag)

Avionics C folds its record format version into the magic word, exactly as the
IMU link folds its protocol version into the sync word, so a reader that does
not know a layout never finds a header for it rather than mis-decoding one.
Format 2 widened alarm_flags from 16 to 32 bits and dropped imu_temp, which had
read zero in every record since IMU link protocol v2 stopped carrying it.

Usage
    parse_logs.py FILE [FILE ...]              summarise, verify CRCs
    parse_logs.py FILE --csv OUT.csv           write every valid record as CSV
    parse_logs.py --pair C_FILE TB_FILE        cross-check the two boards' clocks
"""

import argparse
import struct
import sys
import zlib
from collections import Counter

C_MAGIC = 0x34425053   # "SPB4", Avionics C record format 4 (current)
C3_MAGIC = 0x33425053  # "SPB3", Avionics C record format 3
C2_MAGIC = 0x32425053  # "SPB2", Avionics C record format 2
C1_MAGIC = 0x52425053  # "SPBR", Avionics C record format 1
TB_MAGIC = 0x42544253  # "SBTB"

# ---------------------------------------------------------------------------
# Record layouts. Keep in sync with:
#   Avionics C : main/datalogger.h (TelemetryPacket) + main/flight.h
#   Training Bd: main/tb_log.h (tb_log_record_t)
# ---------------------------------------------------------------------------
# Format 3: scaled integers throughout, three sequence counters. The floats the
# rest of these tools expect are reconstructed by SCALES below, so nothing
# downstream has to know which format a file is in.
C_FMT = ("<I" "HHHH" "hhhH" "BBBB" "I" "3h" "3h" "3h" "II"
         "3h" "h" "H" "h" "ii" "hHH" "I" "BB" "I" "H" "I")
C_FIELDS = [
    "magic", "seq", "sd_seq", "nor_seq", "lora_seq",
    "alt_m", "climb_rate", "baro_alt_m", "mach",
    "phase", "prev_phase", "pyros", "baro_trusted", "alarm_flags",
    "accel_lg_x", "accel_lg_y", "accel_lg_z",
    "accel_hg_x", "accel_hg_y", "accel_hg_z",
    "gyro_x", "gyro_y", "gyro_z",
    "imu_link_t_us", "imu_link_pkt_count",
    "mag_x", "mag_y", "mag_z", "mag_temp",
    "baro_pressure", "baro_temp",
    "gps_lat", "gps_lon",
    "gps_alt_m", "gps_ground_speed_mps", "gps_course_deg",
    "gps_time_utc", "gps_fix_quality", "gps_num_sats",
    "timestamp_since_boot_us", "min_heap_size", "crc32",
]

# field -> (multiply by, then add). Keep in step with the TLM_* scales in
# main/datalogger.h; the datalogger host test round-trips every one of them.
C3_SCALES = {
    "alt_m": (0.2, 0.0), "climb_rate": (0.02, 0.0), "baro_alt_m": (0.2, 0.0),
    "mach": (0.0001, 0.0),
    "accel_lg_x": (0.488, 0.0), "accel_lg_y": (0.488, 0.0), "accel_lg_z": (0.488, 0.0),
    "accel_hg_x": (1.952, 0.0), "accel_hg_y": (1.952, 0.0), "accel_hg_z": (1.952, 0.0),
    "gyro_x": (70.0, 0.0), "gyro_y": (70.0, 0.0), "gyro_z": (70.0, 0.0),
    "mag_x": (0.01, 0.0), "mag_y": (0.01, 0.0), "mag_z": (0.01, 0.0),
    "mag_temp": (0.01, 0.0), "baro_temp": (0.01, 0.0),
    "baro_pressure": (0.0125, 300.0),
    "gps_lat": (1e-7, 0.0), "gps_lon": (1e-7, 0.0),
    "gps_alt_m": (0.5, 0.0), "gps_ground_speed_mps": (0.02, 0.0),
    "gps_course_deg": (0.01, 0.0),
    "min_heap_size": (1024.0, 0.0),
}

# Format 2: uint32 alarm_flags, no imu_temp.
C2_FMT = "<I" "ff ffB BBB I" "6f" "3f" "II" "4f" "2f" "2d" "3f" "I" "BB" "Q" "I" "I"
C2_FIELDS = [
    "magic",
    "alt_m", "climb_rate", "baro_alt_m", "mach", "baro_trusted",
    "phase", "prev_phase", "pyros", "alarm_flags",
    "accel_lg_x", "accel_lg_y", "accel_lg_z", "accel_hg_x", "accel_hg_y", "accel_hg_z",
    "gyro_x", "gyro_y", "gyro_z",
    "imu_link_t_us", "imu_link_pkt_count",
    "mag_x", "mag_y", "mag_z", "mag_temp",
    "baro_pressure", "baro_temp",
    "gps_lat", "gps_lon",
    "gps_alt_m", "gps_ground_speed_mps", "gps_course_deg",
    "gps_time_utc", "gps_fix_quality", "gps_num_sats",
    "timestamp_since_boot_us",
    "min_heap_size", "crc32",
]

# Format 1: uint16 alarm_flags, and an imu_temp that always read zero.
C1_FMT = "<I" "ff ffB BBB H" "6f" "4f" "II" "4f" "2f" "2d" "3f" "I" "BB" "Q" "I" "I"
C1_FIELDS = (C2_FIELDS[:10] + ["accel_lg_x", "accel_lg_y", "accel_lg_z",
                              "accel_hg_x", "accel_hg_y", "accel_hg_z",
                              "gyro_x", "gyro_y", "gyro_z", "imu_temp"]
             + C2_FIELDS[19:])

TB_FMT = "<I HH Q II" "10h" "BBB B" "2f" "4f" "I" "I"
TB_FIELDS = [
    "magic", "len", "version", "t_us", "imu_pkt_count", "imu_t_us",
    "xl_x", "xl_y", "xl_z", "hg_x", "hg_y", "hg_z", "gy_x", "gy_y", "gy_z", "imu_temp_raw",
    "imu_flags", "imu_link_ok", "launched", "_pad",
    "baro_pressure", "baro_temp",
    "mag_x", "mag_y", "mag_z", "mag_temp",
    "sensor_t_ms", "crc32",
]

# Format 3 was format 4 without sd_seq; it existed for an hour on the bench.
C3_FMT = C_FMT.replace("<IHHHH", "<IHHH", 1)
C3_FIELDS = [f for f in C_FIELDS if f != "sd_seq"]
C3_SIZE = struct.calcsize(C3_FMT)

C_SIZE = struct.calcsize(C_FMT)
C2_SIZE = struct.calcsize(C2_FMT)
C1_SIZE = struct.calcsize(C1_FMT)
TB_SIZE = struct.calcsize(TB_FMT)
TB_LOG_PERIOD_US = 10000   # tb_log.h TB_LOG_PERIOD_MS

ALARM_FLAGS = {
    0: "SPI2_INIT_FAIL", 1: "I2C_INIT_FAIL", 2: "BARO_INIT_FAIL", 3: "MAG_INIT_FAIL",
    4: "GPS_INIT_FAIL", 5: "LORA_INIT_FAIL", 6: "PYRO_INIT_FAIL", 7: "SDCARD_INIT_FAIL",
    8: "IMU_INIT_FAIL", 9: "FLASH_INIT_FAIL", 10: "DATALOGGER_INIT_FAIL",
    11: "TASK_SPAWN_FAIL", 12: "IMU_LINK_LOST", 13: "BARO_DISTRUST_TIMEOUT",
    14: "IMU_QUEUE_OVERFLOW", 15: "ACCEL_BACKSTOP", 16: "HG_AXIS_UNAVAILABLE",
    17: "VERT_AXIS_MISMATCH",
}
PHASES = ["PAD", "POWERED", "COAST", "DESCENT"]

# Raw-to-engineering conversions, matching the LSM6DSV80X full scales the
# firmware configures (+-16 g, +-64 g, +-2000 dps).
LG_MG_PER_LSB = 0.488
HG_MG_PER_LSB = 1.952
GY_MDPS_PER_LSB = 70.0


# Derived columns added to every CSV row so the output is usable without
# knowing the sensor full scales.
def engineering_columns(lay, r):
    if lay is TB_LAYOUT:
        return {
            "t_s": r["t_us"] / 1e6,
            "accel_lg_x_mg": r["xl_x"] * LG_MG_PER_LSB,
            "accel_lg_y_mg": r["xl_y"] * LG_MG_PER_LSB,
            "accel_lg_z_mg": r["xl_z"] * LG_MG_PER_LSB,
            "accel_hg_x_mg": r["hg_x"] * HG_MG_PER_LSB,
            "accel_hg_y_mg": r["hg_y"] * HG_MG_PER_LSB,
            "accel_hg_z_mg": r["hg_z"] * HG_MG_PER_LSB,
            "gyro_x_dps": r["gy_x"] * GY_MDPS_PER_LSB / 1000.0,
            "gyro_y_dps": r["gy_y"] * GY_MDPS_PER_LSB / 1000.0,
            "gyro_z_dps": r["gy_z"] * GY_MDPS_PER_LSB / 1000.0,
            "imu_temp_c": r["imu_temp_raw"] / 256.0 + 25.0,
            "imu_t_s": r["imu_t_us"] / 1e6,
        }
    return {
        "t_s": r["timestamp_since_boot_us"] / 1e6,
        "imu_link_t_s": r["imu_link_t_us"] / 1e6,
        "phase_name": PHASES[r["phase"]] if r["phase"] < len(PHASES) else str(r["phase"]),
        "baro_alt_m_raw": r["baro_alt_m"],
        "alarms": "|".join(n for b, n in ALARM_FLAGS.items() if r["alarm_flags"] & (1 << b)),
    }


def crc_candidates(payload):
    """The device uses esp_rom_crc32_le(0, ...). Its pre/post inversion
    convention differs between ROM versions, so accept any standard variant and
    report which one matched."""
    z = zlib.crc32(payload) & 0xFFFFFFFF
    return {
        "zlib": z,
        "zlib_inv": z ^ 0xFFFFFFFF,
        "rom_le": zlib.crc32(payload, 0xFFFFFFFF) & 0xFFFFFFFF,
        "rom_le_inv": (zlib.crc32(payload, 0xFFFFFFFF) & 0xFFFFFFFF) ^ 0xFFFFFFFF,
    }


class Layout:
    def __init__(self, name, magic, fmt, fields, size, scales=None):
        self.name, self.magic, self.fmt, self.fields, self.size = name, magic, fmt, fields, size
        # Scaled-integer formats carry {field: (lsb, offset)}; applying it here
        # means every tool downstream sees engineering units and never has to
        # know which format the file was.
        self.scales = scales or {}


C_LAYOUT = Layout("avionics-c fmt4", C_MAGIC, C_FMT, C_FIELDS, C_SIZE, C3_SCALES)
C3_LAYOUT = Layout("avionics-c fmt3", C3_MAGIC, C3_FMT, C3_FIELDS, C3_SIZE, C3_SCALES)
C2_LAYOUT = Layout("avionics-c fmt2", C2_MAGIC, C2_FMT, C2_FIELDS, C2_SIZE)
C1_LAYOUT = Layout("avionics-c fmt1", C1_MAGIC, C1_FMT, C1_FIELDS, C1_SIZE)
TB_LAYOUT = Layout("training-board", TB_MAGIC, TB_FMT, TB_FIELDS, TB_SIZE)

ALL_LAYOUTS = (C_LAYOUT, C3_LAYOUT, C2_LAYOUT, C1_LAYOUT, TB_LAYOUT)
C_LAYOUTS = (C_LAYOUT, C3_LAYOUT, C2_LAYOUT, C1_LAYOUT)


def detect_layout(data):
    for lay in ALL_LAYOUTS:
        if len(data) >= 4 and struct.unpack_from("<I", data, 0)[0] == lay.magic:
            return lay
    # Not at offset 0 (a truncated first record): scan a little.
    for off in range(0, min(len(data), 4096)):
        for lay in ALL_LAYOUTS:
            if len(data) >= off + 4 and struct.unpack_from("<I", data, off)[0] == lay.magic:
                return lay
    return None


def parse(path):
    """Return (layout, records, stats). Records are dicts of valid records."""
    with open(path, "rb") as f:
        data = f.read()
    lay = detect_layout(data)
    if lay is None:
        return None, [], {"bytes": len(data)}

    records = []
    crc_hits = Counter()
    bad_crc = resync = 0
    i = 0
    # Establish which CRC convention this file uses from the first record whose
    # magic and size line up, then require it for the rest.
    convention = None
    while i + lay.size <= len(data):
        if struct.unpack_from("<I", data, i)[0] != lay.magic:
            i += 1
            resync += 1
            continue
        raw = data[i:i + lay.size]
        vals = struct.unpack(lay.fmt, raw)
        rec = dict(zip(lay.fields, vals))
        # Scaled integers become engineering units right here, so verify_stream,
        # verify_hil, plot_flight and the CSV export all stay format-agnostic.
        for f, (lsb, off) in lay.scales.items():
            rec[f] = rec[f] * lsb + off
        cands = crc_candidates(raw[:-4])
        match = [k for k, v in cands.items() if v == rec["crc32"]]
        if match:
            crc_hits.update(match)
            if convention is None:
                convention = match[0]
            records.append(rec)
            i += lay.size
        else:
            bad_crc += 1
            i += 1
            resync += 1
    return lay, records, {
        "bytes": len(data),
        "bad_crc": bad_crc,
        "resync_bytes": resync,
        "crc_convention": convention,
        "crc_hits": dict(crc_hits),
    }


def summarise(path, lay, recs, st):
    print(f"\n=== {path} ===")
    print(f"  {st['bytes']} bytes")
    if lay is None:
        print("  no recognisable records (magic 'SPB2', 'SPBR' or 'SBTB' not found)")
        return
    print(f"  format          : {lay.name}, {lay.size}-byte records")
    print(f"  valid records   : {len(recs)}")
    print(f"  CRC failures    : {st['bad_crc']}  (bytes skipped while resyncing: {st['resync_bytes']})")
    print(f"  CRC convention  : {st['crc_convention']}")
    if not recs:
        return

    if lay in C_LAYOUTS:
        t = [r["timestamp_since_boot_us"] / 1e6 for r in recs]
        span = t[-1] - t[0]
        print(f"  board clock     : {t[0]:.3f} s -> {t[-1]:.3f} s  (span {span:.1f} s)")
        if span > 0:
            print(f"  record rate     : {len(recs) / span:.1f} Hz")
        alarms = 0
        for r in recs:
            alarms |= r["alarm_flags"]
        print(f"  alarm bits seen : 0x{alarms:04X} " +
              (", ".join(n for b, n in ALARM_FLAGS.items() if alarms & (1 << b)) or "(none)"))
        phases = Counter(PHASES[r["phase"]] if r["phase"] < len(PHASES) else str(r["phase"]) for r in recs)
        print(f"  flight phases   : {dict(phases)}")
        linked = [r for r in recs if r["imu_link_t_us"] != 0]
        print(f"  IMU-link stamps : {len(linked)}/{len(recs)} records carry a Training Board timestamp")
        if linked:
            print(f"  link pkt count  : {linked[0]['imu_link_pkt_count']} -> {linked[-1]['imu_link_pkt_count']}")
        r = recs[-1]
        distrusted = sum(1 for r in recs if not r["baro_trusted"])
        if distrusted:
            print(f"  baro distrusted : {distrusted} records inside the transonic band "
                  f"(peak Mach {max(r['mach'] for r in recs):.2f})")
        print(f"  last record     : alt {r['alt_m']:.1f} m, climb {r['climb_rate']:.2f} m/s, "
              f"accel_lg ({r['accel_lg_x']:.0f}, {r['accel_lg_y']:.0f}, {r['accel_lg_z']:.0f}) mg, "
              f"baro {r['baro_pressure']:.2f} hPa, sats {r['gps_num_sats']}")
    else:
        t = [r["t_us"] / 1e6 for r in recs]
        span = t[-1] - t[0]
        print(f"  board clock     : {t[0]:.3f} s -> {t[-1]:.3f} s  (span {span:.1f} s)")
        if span > 0:
            print(f"  record rate     : {len(recs) / span:.1f} Hz")
        pk = [r["imu_pkt_count"] for r in recs]
        print(f"  IMU frames sent : {pk[0]} -> {pk[-1]}  ({pk[-1] - pk[0]} over the file)")
        if span > 0:
            print(f"  IMU frame rate  : {(pk[-1] - pk[0]) / span:.1f} Hz")
        down = sum(1 for r in recs if not r["imu_link_ok"])
        print(f"  IMU stream idle : {down} records where no frame was sent in the last 100 ms")
        launched = [i for i, r in enumerate(recs) if r["launched"]]
        if launched:
            print(f"  launch detected : at record {launched[0]} (t = {recs[launched[0]]['t_us']/1e6:.3f} s), "
                  f"{len(recs) - launched[0]} records after it")
        else:
            print(f"  launch detected : no (this log is all pad time)")
        r = recs[-1]
        print(f"  last record     : accel_lg ({r['xl_x'] * LG_MG_PER_LSB:.0f}, {r['xl_y'] * LG_MG_PER_LSB:.0f}, "
              f"{r['xl_z'] * LG_MG_PER_LSB:.0f}) mg, gyro ({r['gy_x'] * GY_MDPS_PER_LSB / 1000:.2f}, "
              f"{r['gy_y'] * GY_MDPS_PER_LSB / 1000:.2f}, {r['gy_z'] * GY_MDPS_PER_LSB / 1000:.2f}) dps, "
              f"baro {r['baro_pressure']:.2f} hPa")

    # Monotonic-time check: a backwards jump means records from two sessions
    # ended up in one file, or a record was reconstructed wrongly.
    key = "timestamp_since_boot_us" if lay in C_LAYOUTS else "t_us"
    back = sum(1 for a, b in zip(recs, recs[1:]) if b[key] < a[key])
    if back:
        print(f"  ⚠ {back} backwards time steps (records out of order or two sessions in one file)")


def to_csv(lay, recs, out):
    import csv
    if not recs:
        print(f"  nothing to write to {out}")
        return
    extra = list(engineering_columns(lay, recs[0]).keys())
    fields = lay.fields + [c for c in extra if c not in lay.fields]
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in recs:
            row = dict(r)
            row.update(engineering_columns(lay, r))
            w.writerow(row)
    print(f"  wrote {len(recs)} records to {out}")


def plot(lay, recs, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib is not installed; skipping the plot", file=sys.stderr)
        return
    if not recs:
        return
    cols = [engineering_columns(lay, r) for r in recs]
    t = [c["t_s"] for c in cols]
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    if lay is TB_LAYOUT:
        for k, lbl in (("accel_lg_x_mg", "x"), ("accel_lg_y_mg", "y"), ("accel_lg_z_mg", "z")):
            ax[0].plot(t, [c[k] for c in cols], lw=0.6, label=lbl)
        ax[0].set_ylabel("low-g accel [mg]")
        for k, lbl in (("gyro_x_dps", "x"), ("gyro_y_dps", "y"), ("gyro_z_dps", "z")):
            ax[1].plot(t, [c[k] for c in cols], lw=0.6, label=lbl)
        ax[1].set_ylabel("gyro [dps]")
        ax[2].plot(t, [r["baro_pressure"] for r in recs], lw=0.6)
        ax[2].set_ylabel("pressure [hPa]")
        for k, lbl in (("mag_x", "x"), ("mag_y", "y"), ("mag_z", "z")):
            ax[3].plot(t, [r[k] for r in recs], lw=0.6, label=lbl)
        ax[3].set_ylabel("mag [uT]")
        title = "Training Board log"
    else:
        ax[0].plot(t, [r["alt_m"] for r in recs], lw=0.6)
        ax[0].set_ylabel("altitude [m]")
        ax[1].plot(t, [r["climb_rate"] for r in recs], lw=0.6)
        ax[1].set_ylabel("climb rate [m/s]")
        for k, lbl in (("accel_lg_x", "x"), ("accel_lg_y", "y"), ("accel_lg_z", "z")):
            ax[2].plot(t, [r[k] for r in recs], lw=0.6, label=lbl)
        ax[2].set_ylabel("low-g accel [mg]")
        ax[3].plot(t, [r["phase"] for r in recs], lw=0.8)
        ax[3].set_ylabel("flight phase")
        title = "Avionics C log"
    for a in ax:
        a.grid(alpha=0.3)
        if a.get_legend_handles_labels()[0]:
            a.legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("board clock [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  wrote plot to {out}")


def cross_check(c_path, tb_path):
    """Match Avionics C records to Training Board records through the IMU frame
    timestamp both boards store, and report the clock relationship."""
    import bisect
    c_lay, c_recs, _ = parse(c_path)
    tb_lay, tb_recs, _ = parse(tb_path)
    if c_lay not in C_LAYOUTS or tb_lay is not TB_LAYOUT:
        print("--pair needs one Avionics C file and one Training Board file", file=sys.stderr)
        return 1

    print(f"\n=== time-sync cross-check ===")
    print(f"  Avionics C     : {c_path}  ({len(c_recs)} records)")
    print(f"  Training Board : {tb_path}  ({len(tb_recs)} records)")

    linked = [r for r in c_recs if r["imu_link_t_us"] != 0]
    if not linked or not tb_recs:
        print("  nothing to match (no link timestamps, or an empty Training Board log)")
        return 1
    print(f"  C records with a link stamp     : {len(linked)}/{len(c_recs)}")

    # Both boards sample the SAME frame only about one time in nine (the
    # Training Board logs at 100 Hz and Avionics C at 10 Hz, out of a 937 Hz
    # stream), so alignment quality is the distance to the nearest Training
    # Board record, not the number of exact hits.
    tb_t = sorted(r["imu_t_us"] for r in tb_recs)
    residuals = []
    for r in linked:
        t = r["imu_link_t_us"]
        i = bisect.bisect_left(tb_t, t)
        cands = [tb_t[j] for j in (i - 1, i) if 0 <= j < len(tb_t)]
        if cands:
            residuals.append(min(abs(t - c) for c in cands))
    if residuals:
        residuals.sort()
        med = residuals[len(residuals) // 2]
        p95 = residuals[int(len(residuals) * 0.95)]
        print(f"  nearest TB record, median gap   : {med} us  (95th pct {p95} us, max {residuals[-1]} us)")
        print(f"    (one Training Board record every {TB_LOG_PERIOD_US} us, so a gap below that means every")
        print(f"     Avionics C sample lands inside the Training Board's record stream)")

    # Clock offset: C's own receive time minus the sender's send time.
    offs = [(r["timestamp_since_boot_us"] - r["imu_link_t_us"]) / 1e6 for r in linked]
    print(f"  clock offset (C - TB)           : {offs[0]:+.6f} s -> {offs[-1]:+.6f} s")
    drift = offs[-1] - offs[0]
    span = (linked[-1]["imu_link_t_us"] - linked[0]["imu_link_t_us"]) / 1e6
    if span > 0:
        ppm = drift / span * 1e6 if span else 0.0
        print(f"  drift over {span:.1f} s              : {drift * 1e3:+.1f} ms  ({ppm:+.1f} ppm)")
        # A SIM_REPLAY build puts SIM time in imu_link_t_us instead of the sender's
        # clock, and sim time advances one sample period per sample DELIVERED, so
        # it runs slow by however far real delivery is below the nominal rate. That
        # shows up here as a drift no crystal could have.
        if abs(ppm) > 200:
            print(f"  NOTE: {ppm:+.0f} ppm is far outside any crystal spec. If these logs came from a")
            print( "        SIM_REPLAY build, imu_link_t_us carries sim time rather than the sender's")
            print( "        clock, and the offset, drift and delivery figures above are meaningless.")
    print(f"  => a Training Board record at t_us T aligns with Avionics C time T {offs[-1]:+.6f} s")

    # Delivery, measured ONLY over the window both boards were up. Comparing
    # whole-file totals would count frames the Training Board sent before
    # Avionics C had even booted and report them as losses.
    lo, hi = linked[0]["imu_link_t_us"], linked[-1]["imu_link_t_us"]
    tb_in_window = [r for r in tb_recs if lo <= r["imu_t_us"] <= hi]
    if len(tb_in_window) >= 2:
        sent = tb_in_window[-1]["imu_pkt_count"] - tb_in_window[0]["imu_pkt_count"]
        accepted = linked[-1]["imu_link_pkt_count"] - linked[0]["imu_link_pkt_count"]
        overlap_s = (tb_in_window[-1]["imu_t_us"] - tb_in_window[0]["imu_t_us"]) / 1e6
        print(f"  overlapping window              : {overlap_s:.1f} s")
        print(f"  IMU frames sent by TB in it     : {sent}")
        print(f"  IMU frames accepted by C in it  : {accepted}")
        if sent:
            print(f"  link delivery over that window  : {100.0 * accepted / sent:.2f} %")
    return 0


def compare_same_board(a_path, b_path):
    """Two files from ONE board (the live SD session and the NOR dump of it):
    check that records sharing a timestamp are byte-for-byte the same."""
    a_lay, a_recs, _ = parse(a_path)
    b_lay, b_recs, _ = parse(b_path)
    if a_lay is None or b_lay is None or a_lay is not b_lay:
        print("--same-board needs two files from the same board", file=sys.stderr)
        return 1
    key = "timestamp_since_boot_us" if a_lay in C_LAYOUTS else "t_us"
    print(f"\n=== live SD copy vs NOR copy ===")
    print(f"  A: {a_path}  ({len(a_recs)} records)")
    print(f"  B: {b_path}  ({len(b_recs)} records)")
    by_t = {r[key]: r for r in a_recs}
    shared = mismatched = 0
    for r in b_recs:
        other = by_t.get(r[key])
        if other is None:
            continue
        shared += 1
        if other != r:
            mismatched += 1
    print(f"  records present in both         : {shared}")
    print(f"  of those, differing in any field: {mismatched}")
    if shared and not mismatched:
        print("  => the two storage paths wrote identical data")
    elif not shared:
        print("  => no shared timestamps; are these the same session?")
    return 0

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--csv", metavar="OUT", help="write CSV. With one input this is the output file; "
                                                 "with several it is a directory, one CSV per input "
                                                 "(or the single merged file when --merge is given)")
    ap.add_argument("--merge", action="store_true", help="concatenate all inputs into one time-ordered set")
    ap.add_argument("--plot", metavar="OUT", help="write a PNG overview (one input, or --merge)")
    ap.add_argument("--pair", action="store_true", help="cross-check an Avionics C file against a Training Board file")
    ap.add_argument("--same-board", action="store_true", dest="same_board",
                    help="compare a live SD session against the NOR dump of the same session")
    args = ap.parse_args()

    if args.pair:
        if len(args.files) != 2:
            ap.error("--pair takes exactly two files: the Avionics C log and the Training Board log")
        return cross_check(args.files[0], args.files[1])

    if args.same_board:
        if len(args.files) != 2:
            ap.error("--same-board takes exactly two files from the same board")
        return compare_same_board(args.files[0], args.files[1])

    parsed = []
    for path in args.files:
        lay, recs, st = parse(path)
        summarise(path, lay, recs, st)
        if lay is not None:
            parsed.append((path, lay, recs))
    if not parsed:
        return 1

    if args.merge:
        lays = {lay.name for _, lay, _ in parsed}
        if len(lays) > 1:
            print("--merge needs all inputs to be from the same board", file=sys.stderr)
            return 1
        lay = parsed[0][1]
        key = "timestamp_since_boot_us" if lay in C_LAYOUTS else "t_us"
        merged = sorted((r for _, _, recs in parsed for r in recs), key=lambda r: r[key])
        print(f"\n=== merged: {len(merged)} records from {len(parsed)} files ===")
        parsed = [("merged", lay, merged)]

    if args.csv:
        if len(parsed) == 1:
            to_csv(parsed[0][1], parsed[0][2], args.csv)
        else:
            import os
            os.makedirs(args.csv, exist_ok=True)
            for path, lay, recs in parsed:
                out = os.path.join(args.csv, os.path.basename(path).rsplit(".", 1)[0] + ".csv")
                to_csv(lay, recs, out)

    if args.plot:
        if len(parsed) != 1:
            print("--plot takes one input, or several with --merge", file=sys.stderr)
            return 1
        plot(parsed[0][1], parsed[0][2], args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())

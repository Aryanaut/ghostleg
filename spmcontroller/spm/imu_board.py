"""
imu_board.py - Read the femur-mounted training board over USB and turn its raw
IMU samples into the end-effector's orientation and angular velocity.

    python -m spm.imu_board                 # auto-find the board, print live pose
    python -m spm.imu_board --port COM7 --raw

Pieces (each usable on its own):

  UsbFrameParser   bytes -> frames.  Protocol defined in the firmware's
                   main/usb_stream.h:  A5 96 | type | len | payload | crc16
                   Unframed bytes (bootloader / panic output) are passed on as text.
  ImuBoard         background thread: opens the port, parses frames, scales raw
                   LSB to g / rad/s using the INFO frame, and feeds the estimator.
  OrientationEstimator
                   gyro integration + gravity tilt correction (Mahony-style),
                   gyro bias tracking at standstill.
  MountCalibration solves R_EI (how the board is mounted on the femur) from
                   pairs of {rotation the SPM made, rotation the gyro measured}.

Frames used throughout (same as the rest of the package):
  B  SPM base frame (fixed in the lab), z = ring axis
  E  end effector (femoral head)        R_BE = effector orientation in B
  I  IMU sensor axes (LSM6DSV80X)       R_EI = IMU orientation in E (fixed mount)
  so  R_BI = R_BE @ R_EI   and   R_BE = R_BI @ R_EI.T

What is and is not observable from this IMU
  * Tilt relative to gravity: absolute (accelerometer), no drift.
  * Rotation ABOUT the gravity vector: gyro only, so it drifts slowly with any
    residual gyro bias. Bias is re-estimated whenever the leg is still, and the
    heading can be re-referenced to the encoder pose with align().  The
    magnetometer is streamed but NOT used: it sits a few cm from three stepper
    motors, whose magnets and coil currents would dominate the Earth's field.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .kinematics import axis_angle, rotvec

G = 9.80665
ESPRESSIF_VID = 0x303A        # USB vendor id of the ESP32-S3's built-in USB Serial/JTAG

# ---------------------------------------------------------------------------
# Protocol (mirror of firmware main/usb_stream.h)
# ---------------------------------------------------------------------------
SYNC0, SYNC1 = 0xA5, 0x96
T_IMU, T_MAG, T_TEXT, T_INFO = 0x01, 0x02, 0x03, 0x04
MAX_PAYLOAD = 240
IMU_FMT = "<II3h3h3hB"            # seq, t_us, xl[3], gy[3], hg[3], flags  (27 B)
MAG_FMT = "<I3ff"                 # t_us, mag[3], temp                    (20 B)
INFO_FMT = "<BHfffIII"            # proto, odr, xl/gy/hg scale, dropped, imu_frames, uptime (27 B)


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) - same as the firmware's imu_link_crc16."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ftype: int, payload: bytes) -> bytes:
    """Encode a frame exactly as the firmware does (used by tests and the simulator)."""
    body = bytes([ftype, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


class UsbFrameParser:
    """Incremental byte-stream parser.  feed(bytes) -> list of (type, payload).

    On a bad CRC it drops ONE byte and re-scans, so a corrupted frame costs only
    that frame.  Bytes that are not part of any frame are collected in
    `self.loose_text` (e.g. the ROM bootloader's boot messages)."""

    def __init__(self):
        self.buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.loose_text = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([SYNC0, SYNC1]))
            if i < 0:
                # keep a trailing A5 in case its partner arrives next read
                keep = 1 if self.buf.endswith(bytes([SYNC0])) else 0
                self.loose_text += self.buf[:len(self.buf) - keep]
                del self.buf[:len(self.buf) - keep]
                break
            if i:
                self.loose_text += self.buf[:i]
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            ln = self.buf[3]
            if ln > MAX_PAYLOAD:
                del self.buf[0]                     # impossible length -> false sync
                continue
            total = ln + 6
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:4 + ln])
            crc = self.buf[4 + ln] | (self.buf[5 + ln] << 8)
            if crc16_ccitt_false(body) != crc:
                self.crc_errors += 1
                del self.buf[0]
                continue
            out.append((body[0], body[2:]))
            self.frames_ok += 1
            del self.buf[:total]
        del self.loose_text[:-4096]
        return out


# ---------------------------------------------------------------------------
# Orientation estimator
# ---------------------------------------------------------------------------

@dataclass
class EstimatorConfig:
    kp: float = 0.4                # rad/s per rad of tilt error (accel correction strength)
    ki: float = 0.01               # bias correction from accel (tilt axes only)
    accel_gate_g: float = 0.08     # ignore accel when | |a| - 1 g | exceeds this
    still_gyro_dps: float = 0.8    # standstill: |w| below this ...
    still_accel_g: float = 0.01    # ... and accel std below this ...
    still_window_s: float = 0.5    # ... for this long
    bias_alpha: float = 0.05       # bias update weight per still window


class OrientationEstimator:
    """Keeps R_BI (IMU orientation in the base frame) and the gyro bias.

    Needs two calibration facts (from MountCalibration or a saved file):
      R_EI   IMU axes in the effector frame
      up_B   unit 'up' vector (the direction an accelerometer at rest reads)
             expressed in the base frame
    Until then it still integrates the gyro, so angular velocity is valid
    from the first sample; orientation needs align()."""

    def __init__(self, cfg: EstimatorConfig | None = None):
        self.cfg = cfg or EstimatorConfig()
        self.R_EI = np.eye(3)
        self.up_B = np.array([0.0, 0.0, 1.0])
        self.calibrated = False
        self.R_BI = np.eye(3)
        self.aligned = False
        self.bias = np.zeros(3)                 # rad/s, IMU axes
        self.w_I = np.zeros(3)                  # latest bias-corrected rate, IMU axes
        self.a_I = np.array([0.0, 0.0, 1.0])    # latest accel (g)
        self.last_t = None
        self.still = False
        self._win = deque()                     # (t, w_raw, a) for standstill detection
        self.motion_still_hint = True           # set False by the controller while it commands motion

    # -- persistence -------------------------------------------------------
    def to_dict(self) -> dict:
        return {"R_EI": self.R_EI.tolist(), "up_B": self.up_B.tolist(), "bias": self.bias.tolist()}

    def load(self, d: dict) -> None:
        self.R_EI = np.array(d["R_EI"], float)
        self.up_B = np.array(d["up_B"], float)
        self.bias = np.array(d.get("bias", [0, 0, 0]), float)
        self.calibrated = True

    # -- reference -----------------------------------------------------------
    def align(self, R_BE_ref: np.ndarray) -> None:
        """Declare the effector's current true orientation (e.g. the encoder FK
        pose while still).  Sets heading; tilt is then kept by gravity."""
        self.R_BI = R_BE_ref @ self.R_EI
        self.aligned = True

    # -- main update -------------------------------------------------------------
    def update(self, t: float, gyro_rad_s: np.ndarray, accel_g: np.ndarray) -> None:
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        self.a_I = accel_g
        self._detect_still(t, gyro_rad_s, accel_g)

        w = gyro_rad_s - self.bias
        self.w_I = w
        if not (0 < dt < 0.1):                 # first sample or a gap: don't integrate across it
            return

        c = self.cfg
        corr = np.zeros(3)
        anorm = float(np.linalg.norm(accel_g))
        if self.calibrated and abs(anorm - 1.0) < c.accel_gate_g:
            # Predicted 'up' in IMU axes vs measured.  The cross product is the
            # small rotation (IMU frame) that would bring them together.
            up_pred_I = self.R_BI.T @ self.up_B
            e = np.cross(accel_g / anorm, up_pred_I)
            corr = c.kp * e
            self.bias -= c.ki * e * dt
        self.R_BI = self.R_BI @ axis_angle(w + corr, float(np.linalg.norm(w + corr)) * dt)
        # Re-orthonormalise occasionally (cheap, keeps R a rotation)
        if np.random.random() < 0.01:
            u, _, vt = np.linalg.svd(self.R_BI)
            self.R_BI = u @ vt

    def _detect_still(self, t, w_raw, a):
        c = self.cfg
        self._win.append((t, w_raw.copy(), a.copy()))
        while self._win and t - self._win[0][0] > c.still_window_s:
            self._win.popleft()
        if len(self._win) < 10 or t - self._win[0][0] < 0.9 * c.still_window_s:
            self.still = False
            return
        W = np.array([x[1] for x in self._win])
        A = np.array([x[2] for x in self._win])
        w_mean = W.mean(axis=0)
        still = (np.degrees(np.linalg.norm(w_mean - self.bias)) < c.still_gyro_dps
                 and float(A.std(axis=0).max()) < c.still_accel_g
                 and self.motion_still_hint)
        self.still = bool(still)               # plain bool (numpy bool_ is not JSON-serialisable)
        if still:
            # At rest the gyro should read zero: whatever it reads is bias.
            self.bias += c.bias_alpha * (w_mean - self.bias)
            self._win.clear()                   # one update per window

    # -- outputs -----------------------------------------------------------------
    @property
    def R_BE(self) -> np.ndarray:
        return self.R_BI @ self.R_EI.T

    @property
    def w_E(self) -> np.ndarray:
        """Angular velocity in effector (body) axes, rad/s."""
        return self.R_EI @ self.w_I

    @property
    def w_B(self) -> np.ndarray:
        """Angular velocity in base axes, rad/s."""
        return self.R_BI @ self.w_I


# ---------------------------------------------------------------------------
# Mount calibration: solve R_EI from rotation pairs (no time sync needed)
# ---------------------------------------------------------------------------

class MountCalibration:
    """Collect pairs of rotation vectors for the same physical motion:
        dE = rotation the effector made, in effector axes (from encoder FK)
        dI = rotation the gyro integrated over the same interval, in IMU axes
    For a rigid mount dE = R_EI @ dI, so R_EI is the rotation that best maps
    the dI's onto the dE's - Wahba's problem, solved by SVD (Kabsch).
    Using whole moves between standstills means PC/IMU clock offsets and USB
    latency do not matter."""

    def __init__(self):
        self.pairs: list[tuple[np.ndarray, np.ndarray]] = []

    def add(self, dE: np.ndarray, dI: np.ndarray) -> None:
        self.pairs.append((np.asarray(dE, float), np.asarray(dI, float)))

    def solve(self) -> tuple[np.ndarray, dict]:
        if len(self.pairs) < 2:
            raise ValueError("need at least two rotations about different axes")
        E = np.array([p[0] for p in self.pairs])
        I = np.array([p[1] for p in self.pairs])
        # Need rotation axes spanning >= 2 directions or R is not unique.
        s = np.linalg.svd(E, compute_uv=False)
        if s[1] < 0.1 * s[0]:
            raise ValueError("calibration rotations are all about the same axis")
        H = I.T @ E                                       # 3x3 cross-covariance
        u, _, vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(vt.T @ u.T))
        R = vt.T @ np.diag([1, 1, d]) @ u.T               # R @ dI ~= dE
        resid = [math.degrees(float(np.linalg.norm(R @ i - e))) for e, i in self.pairs]
        scale = [float(np.linalg.norm(i) / max(np.linalg.norm(e), 1e-9)) for e, i in self.pairs]
        return R, {"residual_deg": resid, "gyro_scale": scale, "n": len(self.pairs)}


# ---------------------------------------------------------------------------
# The board: serial thread + scaling + estimator
# ---------------------------------------------------------------------------

def find_board_port() -> str | None:
    """First serial port with Espressif's USB vendor id."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == ESPRESSIF_VID:
            return p.device
    return None


class SampleClock:
    """Works out the IMU's TRUE sample period, and gives every sample a
    timestamp built from it.

    Why this exists (HANDOFF section 6.1)
    -------------------------------------
    The firmware stamps each sample with a clock that advances a NOMINAL
    1e6/240 us per sample.  The LSM6DSV80X runs from its own internal
    oscillator, and on this board it really delivers ~230.5 Hz (board stats
    "tx 230.x Hz", host count 230.8 Hz).  Integrating gyro with the nominal
    4.167 ms instead of the real ~4.34 ms makes every integrated angle ~4%
    too small.  The firmware also re-anchors its clock whenever it falls
    more than 250 ms behind real time, which shows up as a ~254 ms jump in
    t_us every ~6 s, with no samples missing.  So t_us cannot be trusted
    for dt at all.

    What it does instead
    --------------------
    The sequence number counts every sample the sensor produced (gaps
    included), so  production_time(seq) = t0 + seq * T  exactly, with T the
    unknown true period.  We observe each sample a little after it was
    produced:  t_seen = t0 + seq * T + latency,  latency >= 0  (USB, OS,
    and Python thread scheduling, sometimes tens of ms in a burst).
      1. Split the stream into short buckets (bucket_s) and keep, per
         bucket, the sample with the SMALLEST  t_seen - seq * T_nominal.
         That is the sample that arrived quickest, so it sits on the
         'lower envelope' where latency is close to its minimum.  This
         throws away nearly all of the jitter and every late burst.
      2. Fit a straight line  t = a + T * seq  through those minima over a
         sliding window (window_s), by least squares.  Points far above
         the first fit (a bucket whose best sample was still late) are
         dropped and the line is fitted again.
      3. The slope T is the true period.  Error ~ (envelope jitter) / (span),
         e.g. 1 ms over 10 s = 0.01%.
    Until the span is long enough (min_span_s) we fall back to the nominal
    period and report converged = False; mount calibration waits for it.

    Timestamps handed to the estimator are built as  t += (seq step) * T,
    which keeps dt the true sample spacing, never jumps when the
    firmware re-anchors, and spans seq gaps correctly (dt grows, and the
    estimator refuses to integrate across gaps > 0.1 s as before).

    The reference time `t_seen` is whatever monotonic clock the caller
    gives (PC time.perf_counter() in ImuBoard).  The PC and ESP32 crystals
    agree to tens of ppm, which is negligible here."""

    def __init__(self, nominal_hz: float = 240.0, bucket_s: float = 0.25,
                 window_s: float = 60.0, min_span_s: float = 2.0, max_dev: float = 0.2):
        self.nominal_period = 1.0 / nominal_hz
        self.bucket_s = bucket_s          # length of one lower-envelope bucket
        self.window_s = window_s          # how much history the line fit uses
        self.min_span_s = min_span_s      # need this much history before trusting the fit
        self.max_dev = max_dev            # reject fits more than +-20% from nominal (nonsense)
        self.reset()

    def reset(self) -> None:
        """Forget everything (call when the port is re-opened)."""
        self.period = self.nominal_period  # current best estimate of the true period, s
        self.converged = False
        self.fit_rms_ms = float("nan")     # rms scatter of the envelope points about the line
        self._buckets: deque = deque()     # completed (seq, t_seen) envelope points
        self._cur = None                   # (bucket_index, seq, t_seen, key) of the open bucket
        self._seq_unwrapped = None         # seq with 32-bit wrap removed
        self._seq_last_raw = None
        self.t = 0.0                       # corrected sample time, s (starts at 0 per connection)

    def set_nominal(self, nominal_hz: float) -> None:
        """INFO frame tells us the firmware's nominal rate; only matters before convergence."""
        if nominal_hz > 0:
            self.nominal_period = 1.0 / nominal_hz
            if not self.converged:
                self.period = self.nominal_period

    def update(self, seq_raw: int, t_seen: float) -> float:
        """Feed one sample's 32-bit seq and the time it was seen; return its
        corrected timestamp (s)."""
        # ---- unwrap the 32-bit sequence counter -------------------------------
        if self._seq_last_raw is None:
            step = 0
            self._seq_unwrapped = 0                 # this connection's first sample = 0
        else:
            step = (seq_raw - self._seq_last_raw) & 0xFFFFFFFF
            if step >= 2**31:                       # went BACKWARDS = board rebooted
                self.reset()
                return self.update(seq_raw, t_seen)
            self._seq_unwrapped += step
        self._seq_last_raw = seq_raw
        n = self._seq_unwrapped

        # ---- corrected timestamp: advance by (samples elapsed) * true period ---
        self.t += step * self.period

        # ---- lower-envelope bucketing ------------------------------------------
        # key = how late this sample is relative to a nominal-rate line.  Any
        # fixed slope works for picking the minimum inside a 0.25 s bucket.
        key = t_seen - n * self.nominal_period
        b = int(t_seen // self.bucket_s)
        if self._cur is None or b != self._cur[0]:
            if self._cur is not None:               # close the previous bucket
                self._buckets.append((self._cur[1], self._cur[2]))
                self._trim_and_fit()
            self._cur = (b, n, t_seen, key)
        elif key < self._cur[3]:                    # quicker arrival in the same bucket
            self._cur = (b, n, t_seen, key)
        return self.t

    def _trim_and_fit(self) -> None:
        # keep only the last window_s of envelope points
        while self._buckets and self._buckets[-1][1] - self._buckets[0][1] > self.window_s:
            self._buckets.popleft()
        if len(self._buckets) < 4 or self._buckets[-1][1] - self._buckets[0][1] < self.min_span_s:
            return
        P = np.array(self._buckets, float)
        seq, t = P[:, 0], P[:, 1]
        seq_c = seq - seq.mean()                    # centre for numerical conditioning
        T, a = np.polyfit(seq_c, t, 1)
        resid = t - (a + T * seq_c)
        # Second pass: drop buckets whose best sample was still clearly late
        # (whole bucket inside a host stall).  Latency only ever ADDS time,
        # so outliers are one-sided: above the line.
        keep = resid < max(0.002, 3 * float(np.median(np.abs(resid))))
        if keep.sum() >= 4:
            T, a = np.polyfit(seq_c[keep], t[keep], 1)
            resid = t[keep] - (a + T * seq_c[keep])
        if abs(T / self.nominal_period - 1) > self.max_dev:
            return                                  # nonsense (e.g. data burst after a stall); keep old value
        self.period = float(T)
        self.fit_rms_ms = float(np.sqrt(np.mean(resid ** 2)) * 1e3)
        self.converged = True

    @property
    def rate_hz(self) -> float:
        return 1.0 / self.period


@dataclass
class ImuStats:
    imu_frames: int = 0
    mag_frames: int = 0
    seq_gaps: int = 0
    crc_errors: int = 0
    rate_hz: float = 0.0              # frames counted in the last 1 s of PC time (display only)
    board_dropped: int = 0
    board_uptime_s: float = 0.0
    last_rx: float = 0.0


class ImuBoard:
    """Owns the USB port.  Thread-safe accessors for the latest state.

    Opening an ESP32-S3 USB Serial/JTAG port with DTR/RTS asserted can reset
    the chip (those lines drive its auto-reset logic), so both are forced low
    BEFORE the port is opened - the same thing idf.py monitor does."""

    def __init__(self, port: str | None = None, estimator: OrientationEstimator | None = None,
                 cal_path: Path | None = None):
        self.port_name = port
        self.est = estimator or OrientationEstimator()
        self.cal_path = Path(cal_path) if cal_path else None
        if self.cal_path and self.cal_path.exists():
            self.est.load(json.loads(self.cal_path.read_text()))
        self.parser = UsbFrameParser()
        self.stats = ImuStats()
        self.lock = threading.RLock()
        # scale factors: defaults = this firmware's FS (16 g / 2000 dps), replaced by INFO
        self.xl_g_per_lsb = 0.488e-3
        self.gy_rad_per_lsb = math.radians(70e-3)
        self.odr_hz = 240
        self.got_info = False
        self.text_lines: deque[str] = deque(maxlen=200)
        self._text_buf = ""
        self.mag_uT = None
        self.seq = None
        # Two clocks per sample:
        #   t_board  the firmware's t_us, unwrapped.  NOMINAL 240 Hz spacing and
        #            jumps of ~250 ms when the firmware re-anchors it - kept only
        #            for diagnostics / logging, NEVER used for integration.
        #   t        seq * (true period from SampleClock) - what the estimator,
        #            gyro integrator and sample_listeners get.  (HANDOFF 6.1)
        self.t_board = 0.0                     # unwrapped board time, s
        self._t_last_raw = None
        self._t_wraps = 0
        self.clock = SampleClock(self.odr_hz)
        self.t = 0.0
        self._t_rx = 0.0                       # PC arrival time of the bytes being parsed
        self.sample_listeners = []             # callables(t, gyro_rad_s, accel_g) for recorders
        self._rate_win = deque()
        self._ser = None
        self._stop = threading.Event()
        self._thread = None

    # ---- lifecycle ---------------------------------------------------------
    def open(self, port: str | None = None, port_obj=None):
        if port_obj is not None:                 # simulator (spm/imu_sim.py)
            self.clock.reset()
            self._ser = port_obj
            self.port_name = "SIM"
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True, name="imu_board")
            self._thread.start()
            return "SIM"
        import serial
        name = port or self.port_name or find_board_port()
        if not name:
            raise RuntimeError("No Espressif USB device found (VID 303A). Is the board plugged in?")
        ser = serial.Serial()
        ser.port = name
        ser.baudrate = 115200           # ignored by USB CDC, required by the API
        ser.timeout = 0.05
        ser.dtr = False                 # do not touch the chip's reset/boot lines
        ser.rts = False
        ser.open()
        # The board keeps writing into its USB buffer while no program has the
        # port open, so the first bytes after opening can be frames from long
        # ago (seen on hardware: an 80 s timestamp jump, and right after boot
        # the stale frames even included the sensor's settling transient).
        # Throw away everything that arrives in the first 0.3 s.
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            ser.read(65536)
        self.parser = UsbFrameParser()
        self.seq = None
        self._t_last_raw = None
        self.est.last_t = None
        self.clock.reset()                       # true-rate fit starts afresh on every connection
        self._ser = ser
        self.port_name = name
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="imu_board")
        self._thread.start()
        return name

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and time.monotonic() - self.stats.last_rx < 1.0

    # ---- reader thread -------------------------------------------------------
    def _run(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(4096)
            except Exception as exc:               # unplugged
                self._log_text(f"[host] serial error: {exc}")
                break
            if data:
                # Stamp the bytes the moment they arrive: SampleClock uses
                # the quickest arrivals to measure the sensor's true rate.
                self.feed(data, time.perf_counter())

    def feed(self, data: bytes, t_rx: float | None = None) -> None:
        """Process raw bytes (public so tests / simulators can inject data).
        t_rx = PC time (s, monotonic) at which `data` was read; tests pass a
        synthetic time so they can replay a stream faster than real time."""
        if t_rx is None:
            t_rx = time.perf_counter()
        self._t_rx = t_rx
        frames = self.parser.feed(data)
        with self.lock:
            self.stats.crc_errors = self.parser.crc_errors
            if self.parser.loose_text:
                self._log_text(self.parser.loose_text.decode("utf-8", "replace"))
                self.parser.loose_text.clear()
            for ftype, p in frames:
                self.stats.last_rx = time.monotonic()
                if ftype == T_IMU and len(p) == 27:
                    self._on_imu(struct.unpack(IMU_FMT, p))
                elif ftype == T_MAG and len(p) == 20:
                    v = struct.unpack(MAG_FMT, p)
                    self.mag_uT = np.array(v[1:4])
                    self.stats.mag_frames += 1
                elif ftype == T_TEXT:
                    self._log_text(p.decode("utf-8", "replace"))
                elif ftype == T_INFO and len(p) == 27:
                    proto, odr, xl, gy, hg, dropped, nimu, up_ms = struct.unpack(INFO_FMT, p)
                    self.xl_g_per_lsb = xl * 1e-3
                    self.gy_rad_per_lsb = math.radians(gy * 1e-3)
                    self.odr_hz = odr
                    self.clock.set_nominal(odr)       # only used until the true-rate fit converges
                    self.got_info = True
                    self.stats.board_dropped = dropped
                    self.stats.board_uptime_s = up_ms / 1000

    def _log_text(self, s: str):
        self._text_buf += s
        while "\n" in self._text_buf:
            line, self._text_buf = self._text_buf.split("\n", 1)
            line = line.strip("\r")
            if line.strip():
                self.text_lines.append(line)

    def _on_imu(self, v):
        seq, t_us = v[0], v[1]
        xl = np.array(v[2:5], float) * self.xl_g_per_lsb
        gy = np.array(v[5:8], float) * self.gy_rad_per_lsb
        # sequence gaps = samples lost somewhere (USB ring full, host too slow)
        if self.seq is not None and seq != (self.seq + 1) & 0xFFFFFFFF:
            self.stats.seq_gaps += (seq - self.seq - 1) & 0xFFFFFFFF
        self.seq = seq
        # unwrap the 32-bit microsecond clock (wraps every ~71.6 min)
        if self._t_last_raw is not None and t_us < self._t_last_raw and self._t_last_raw - t_us > 2**31:
            self._t_wraps += 1
        self._t_last_raw = t_us
        self.t_board = (self._t_wraps * 2**32 + t_us) * 1e-6
        # Corrected time from the sequence number and the measured true period
        self.t = self.clock.update(seq, self._t_rx)
        self.est.update(self.t, gy, xl)
        self.stats.imu_frames += 1
        now = time.monotonic()
        self._rate_win.append(now)
        while self._rate_win and now - self._rate_win[0] > 1.0:
            self._rate_win.popleft()
        self.stats.rate_hz = len(self._rate_win)
        for cb in self.sample_listeners:
            cb(self.t, gy, xl)

    # ---- calibration helpers --------------------------------------------------
    def gyro_integrator(self):
        """Return an object that integrates bias-corrected gyro from now on;
        call .rotvec() to get the accumulated rotation (IMU axes)."""
        board = self

        class _Integ:
            def __init__(self):
                self.R = np.eye(3)
                self.t = None
                board.sample_listeners.append(self._cb)

            def _cb(self, t, gy, xl):
                if self.t is not None and 0 < t - self.t < 0.1:
                    w = gy - board.est.bias
                    self.R = self.R @ axis_angle(w, float(np.linalg.norm(w)) * (t - self.t))
                self.t = t

            def stop(self):
                if self._cb in board.sample_listeners:
                    board.sample_listeners.remove(self._cb)
                return rotvec(self.R)

        return _Integ()

    def mean_still(self, seconds: float = 2.0):
        """Average accel (g) and gyro (rad/s) over `seconds` - call while still."""
        acc, gyr = [], []
        cb = lambda t, g, a: (gyr.append(g), acc.append(a))
        self.sample_listeners.append(cb)
        time.sleep(seconds)
        self.sample_listeners.remove(cb)
        if len(acc) < 10:
            raise RuntimeError("no IMU samples arriving")
        return np.mean(acc, axis=0), np.mean(gyr, axis=0)

    def save_calibration(self):
        if self.cal_path:
            self.cal_path.write_text(json.dumps(self.est.to_dict(), indent=2))

    # ---- snapshot -----------------------------------------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            e = self.est
            return {
                "port": self.port_name, "connected": self.connected, "got_info": self.got_info,
                "calibrated": e.calibrated, "aligned": e.aligned, "still": e.still,
                "rate_hz": self.stats.rate_hz, "frames": self.stats.imu_frames,
                # true sensor rate from SampleClock (what integration uses)
                "true_rate_hz": round(self.clock.rate_hz, 3), "rate_converged": self.clock.converged,
                "rate_fit_rms_ms": None if math.isnan(self.clock.fit_rms_ms) else round(self.clock.fit_rms_ms, 3),
                "seq_gaps": self.stats.seq_gaps, "crc_errors": self.stats.crc_errors,
                "board_dropped": self.stats.board_dropped, "uptime_s": self.stats.board_uptime_s,
                "accel_g": e.a_I.tolist(), "gyro_dps": np.degrees(e.w_I).tolist(),
                "bias_dps": np.degrees(e.bias).tolist(),
                "w_E_dps": np.degrees(e.w_E).tolist(), "w_B_dps": np.degrees(e.w_B).tolist(),
                "mag_uT": None if self.mag_uT is None else self.mag_uT.tolist(),
                "text": list(self.text_lines)[-8:],
            }


# ---------------------------------------------------------------------------
# CLI: read the board on its own (no SPM needed)
# ---------------------------------------------------------------------------

def _cli():
    import argparse
    ap = argparse.ArgumentParser(description="Read the femur IMU board over USB")
    ap.add_argument("--port", help="serial port (default: auto-detect Espressif VID 303A)")
    ap.add_argument("--raw", action="store_true", help="print raw scaled sensor values")
    ap.add_argument("--hz", type=float, default=5, help="print rate")
    ap.add_argument("--cal", default=str(Path(__file__).resolve().parents[1] / "imu_cal.json"),
                    help="mount calibration file (written by the GUI's IMU calibration)")
    args = ap.parse_args()

    board = ImuBoard(args.port, cal_path=Path(args.cal))
    name = board.open()
    print(f"Reading {name}  (Ctrl+C to stop).  Calibration: "
          f"{'loaded' if board.est.calibrated else 'none - orientation is relative to power-on'}")
    if board.est.calibrated:
        print("Hold the leg still at home for 1 s to align...")
    t0 = time.monotonic()
    shown = 0
    try:
        while True:
            time.sleep(1 / args.hz)
            s = board.snapshot()
            for line in list(board.text_lines)[shown:]:
                print("  [board]", line)
            shown = len(board.text_lines)
            if not board.est.aligned and time.monotonic() - t0 > 1.0:
                board.est.align(np.eye(3))      # without the SPM, 'home' = where it is now
            if args.raw:
                print(f"{s['rate_hz']:5.0f} Hz (true {s['true_rate_hz']:7.2f}{'' if s['rate_converged'] else '?'})  acc[g]={np.round(s['accel_g'], 4)}  "
                      f"gyro[dps]={np.round(s['gyro_dps'], 2)}  mag[uT]={s['mag_uT'] and np.round(s['mag_uT'], 1)}  "
                      f"gaps={s['seq_gaps']} crc={s['crc_errors']}")
            else:
                R = board.est.R_BE
                rv = np.degrees(rotvec(R))
                tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
                print(f"{s['rate_hz']:5.0f} Hz | R_BE rotvec[deg]={np.round(rv, 2)} tilt={tilt:6.2f} | "
                      f"w_E[dps]={np.round(s['w_E_dps'], 2)} | still={s['still']} "
                      f"bias[dps]={np.round(s['bias_dps'], 3)}")
    except KeyboardInterrupt:
        pass
    finally:
        board.close()


if __name__ == "__main__":
    _cli()

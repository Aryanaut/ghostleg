"""
imu_sim.py - A software stand-in for the femur IMU board, for testing the
whole IMU pipeline (frames -> parser -> estimator -> calibration -> trim)
with the simulated motor drivers.  Select it with port "SIM" in the GUI.

It reads the SIMULATED motors' shaft positions (so it sees the true motion,
not the commanded one), runs forward kinematics, and synthesises what an IMU
bolted to the femur at an unknown angle would measure: gyro with bias and
noise, accelerometer = gravity in sensor axes + noise.  It can also add a
fixed "compliance" error between where the linkage says the femur is and
where it really is - exactly the kind of error only the IMU can see.

Timing is simulated the way the REAL board behaves (HANDOFF 6.1): the sensor
samples at its own true rate (true_hz, 230.5 Hz measured on the real board)
while the firmware stamps t_us on a nominal 1e6/240 us-per-sample clock and
re-anchors that clock forward whenever it falls > 250 ms behind.  So a host
that trusted t_us would integrate ~4% short here too, as on hardware.
"""

from __future__ import annotations

import math
import struct
import threading
import time

import numpy as np

from .imu_board import IMU_FMT, INFO_FMT, T_IMU, T_INFO, build_frame
from .kinematics import axis_angle, rotvec


class SimImuPort:
    """Duck-types the bits of serial.Serial that ImuBoard uses."""

    def __init__(self, ctl, mount_rotvec=(0.7, -1.2, 2.0), up_B=(0.35, -0.6, 0.72),
                 bias_dps=(0.4, -0.3, 0.25), compliance_deg=(1.5, 0.0, 0.0), odr=240,
                 true_hz=230.5):
        self.ctl = ctl
        self.R_EI = axis_angle(mount_rotvec, float(np.linalg.norm(mount_rotvec)))
        u = np.asarray(up_B, float)
        self.up_B = u / np.linalg.norm(u)
        self.bias = np.radians(bias_dps)
        # compliance error in effector axes, scaled with tilt from home (a sagging link)
        self.compliance = np.radians(compliance_deg)
        self.odr = odr                 # NOMINAL rate: what the firmware labels / reports in INFO
        self.true_hz = true_hz         # what the sensor really delivers
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.rng = np.random.default_rng(0)
        self.seq = 0
        self._stop = False
        self.R_prev = None
        threading.Thread(target=self._run, daemon=True).start()

    # -- true pose of the simulated rig ---------------------------------
    def _true_pose(self, R_guess):
        c = self.ctl
        port = getattr(c.bus, "port", None) if c.bus else None
        if port is None or not hasattr(port, "motors") or not c.homed:
            return c.R_cmd.copy()
        with port._lock:                         # advance the sim motors to "now" (else they move in 50 Hz steps)
            for mot in port.motors.values():
                mot.update()
        counts = [port.motors[m["address"]].axis - port.motors[m["address"]].zero for m in c.cfg["motors"]]
        th = c._counts_to_theta(counts)
        R, ok = c.spm.fk(th, R_guess)
        if not ok:
            return R_guess
        tilt = rotvec(c.hip.R_neutral.T @ R)
        sag = axis_angle(self.compliance, float(np.linalg.norm(self.compliance)) * min(1.0, np.linalg.norm(tilt) / math.radians(20)))
        return R @ sag

    def _run(self):
        dt = 1.0 / self.true_hz                  # TRUE sample spacing (real time + physics)
        nominal_us = 1000000 // self.odr         # firmware's assumed spacing, as in bridge.c
        t_us = 0                                 # firmware's free-running sample clock
        t0 = time.perf_counter()
        next_t = t0
        n = 0
        while not self._stop:
            R_BE = self._true_pose(self.R_prev if self.R_prev is not None else self.ctl.R_cmd)
            R_BI = R_BE @ self.R_EI
            if self.R_prev is None:
                w_I = np.zeros(3)
            else:
                # body rate from the change in IMU orientation over one sample
                w_I = rotvec((self.R_prev @ self.R_EI).T @ R_BI) / dt
            self.R_prev = R_BE
            gyro = w_I + self.bias + self.rng.normal(0, math.radians(0.07), 3)
            acc = R_BI.T @ self.up_B + self.rng.normal(0, 0.0005, 3)
            xl = np.clip(np.round(acc / 0.488e-3), -32768, 32767).astype(int)
            gy = np.clip(np.round(np.degrees(gyro) / 70e-3), -32768, 32767).astype(int)
            frame = build_frame(T_IMU, struct.pack(IMU_FMT, self.seq, t_us & 0xFFFFFFFF,
                                                   *xl, *gy, 0, 0, 0, 7))
            if n % self.odr == 0:
                up_ms = int((time.perf_counter() - t0) * 1000)     # INFO uptime = real (esp_timer) time
                frame += build_frame(T_INFO, struct.pack(INFO_FMT, 1, self.odr, 0.488, 70.0, 1.952, 0, self.seq, up_ms))
            with self.lock:
                self.buf += frame
                del self.buf[:-200000]
            self.seq += 1
            n += 1
            # bridge.c: advance the nominal clock one period per sample, and
            # re-anchor it to real time if it has fallen > 250 ms behind.
            t_us += nominal_us
            real_us = int((time.perf_counter() - t0) * 1e6)
            if real_us - t_us > nominal_us * self.odr // 4:
                t_us = real_us
            next_t += dt
            sl = next_t - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            elif sl < -0.5:
                # Hopelessly behind (PC suspended / debugger): give up on the
                # lost time rather than emit a huge burst.
                next_t = time.perf_counter()
            # else: slightly behind -> produce the next samples straight away
            # (a burst), like the real sensor's FIFO after a late read.  The
            # average rate then stays exactly true_hz, as on hardware; the old
            # "reset the schedule whenever late" made the sim lose time on
            # Windows (coarse sleep) and run at ~229 Hz instead of 230.5.

    # -- serial-like API ----------------------------------------------------
    def read(self, n):
        time.sleep(0.005)
        with self.lock:
            out = bytes(self.buf[:n])
            del self.buf[:n]
        return out

    def close(self):
        self._stop = True

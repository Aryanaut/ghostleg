"""
imu_control.py - Ties the femur IMU board (imu_board.py) into the controller.

Responsibilities
  * connect / disconnect the board
  * MOUNT CALIBRATION, driven by the rig itself: hold still at neutral, then
    rotate the effector about its own x, y and z axes (out and back), and pair
    each move's encoder-measured rotation with the gyro-integrated rotation.
    Solves R_EI (board -> femur) and the gravity direction in the base frame,
    and saves them to imu_cal.json.
  * ALIGN: re-reference the IMU heading to the encoder pose while still
    (tilt never needs this - gravity provides it continuously)
  * COMPARE: commanded vs encoder-FK vs IMU orientation, every snapshot
  * TRIM (optional closed loop): integrate the IMU-vs-target TILT error into a
    small correction applied to the commanded pose, to take out what the
    motor encoders cannot see (gear backlash, link compliance, geometry error).
    Heading is excluded on purpose: it is the one axis the IMU can't measure
    absolutely, so feeding it back would steer the leg by gyro drift.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np

from .imu_board import ImuBoard, MountCalibration, find_board_port
from .kinematics import axis_angle, rotvec


def ang_deg(R: np.ndarray) -> float:
    """Magnitude of a rotation matrix, degrees."""
    return math.degrees(float(np.linalg.norm(rotvec(R))))


class ImuIntegration:
    def __init__(self, ctl, cal_path: Path):
        self.ctl = ctl                     # the Controller
        self.cal_path = Path(cal_path)
        self.board: ImuBoard | None = None
        self.trim = np.zeros(3)            # rotation vector (base frame) added to the command
        self.cal_thread: threading.Thread | None = None
        self.cal_status = "idle"
        self.cal_result: dict | None = None

    # ------------------------------------------------------------------
    def connect(self, port: str | None):
        self.disconnect()
        if port and port.upper() == "SIM":
            from .imu_sim import SimImuPort
            # simulated board: its own calibration file so it never overwrites the real one
            self.board = ImuBoard(None, cal_path=self.cal_path.with_name("imu_cal_sim.json"))
            name = self.board.open(port_obj=SimImuPort(self.ctl))
        else:
            self.board = ImuBoard(port or None, cal_path=self.cal_path)
            name = self.board.open()
        self.ctl.log(f"IMU board opened on {name}; calibration "
                     f"{'loaded from ' + self.cal_path.name if self.board.est.calibrated else 'not done yet'}")
        if self.board.est.calibrated:
            threading.Thread(target=self._auto_align, daemon=True).start()

    def _auto_align(self):
        time.sleep(1.5)                    # let the filter settle
        try:
            self.align()
        except Exception as exc:
            self.ctl.log(f"IMU auto-align skipped: {exc}")

    def disconnect(self):
        if self.board:
            self.board.close()
        self.board = None
        self.trim[:] = 0

    @property
    def ok(self) -> bool:
        return bool(self.board and self.board.connected)

    def reference_pose(self) -> np.ndarray:
        """Best available 'true' effector pose: encoder FK if we have it, else the command."""
        c = self.ctl
        return c.R_meas if c.R_meas is not None else c.R_cmd

    def align(self):
        if not self.ok:
            raise RuntimeError("IMU board not connected")
        if not self.board.est.calibrated:
            raise RuntimeError("Calibrate the IMU mount first")
        with self.ctl.lock:
            R_ref = self.reference_pose().copy()
        self.board.est.align(R_ref)
        self.trim[:] = 0
        self.ctl.log("IMU heading aligned to the encoder pose")

    # ------------------------------------------------------------------
    # Mount calibration (runs in its own thread; the control loop keeps going)
    # ------------------------------------------------------------------
    def start_calibration(self, tilt_deg: float = 15.0, twist_deg: float = 25.0):
        c = self.ctl
        if not self.ok:
            raise RuntimeError("IMU board not connected / not streaming")
        if not (c.connected and c.homed and c.armed):
            raise RuntimeError("Connect, home and ARM the rig first - calibration moves the leg")
        if self.cal_thread and self.cal_thread.is_alive():
            raise RuntimeError("Calibration already running")
        self.cal_thread = threading.Thread(target=self._calibrate, args=(tilt_deg, twist_deg), daemon=True)
        self.cal_thread.start()

    def _goto(self, R_target, timeout=15.0):
        """Command an effector orientation and wait until the rig is there and still."""
        c = self.ctl
        with c.lock:
            c.modes["effector"].R_BE = R_target
            c.mode = c.modes["effector"]
            c.ui_seq += 1
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not c.armed:
                raise RuntimeError("rig disarmed / E-stopped during calibration")
            with c.lock:
                cmd_done = ang_deg(R_target @ c.R_cmd.T) < 0.02
                meas_ok = c.R_meas is None or ang_deg(R_target @ c.R_meas.T) < 0.5
            if cmd_done and meas_ok:
                time.sleep(0.8)            # settle: let vibration die out
                return
            time.sleep(0.02)
        raise RuntimeError("timed out waiting for the rig to reach a calibration pose")

    def _safe_swing(self, R_home, axis, sign, ang_req, min_deg=4.0, step=1.0):
        """Largest angle a <= ang_req (deg) such that every pose on the swing
        R_home -> R_home @ rot(axis, sign*a), checked each `step` degrees, passes
        the controller's safety gate.  None if not even min_deg is possible."""
        c = self.ctl
        with c.lock:
            th = c.theta_cmd.copy()
            best = None
            for k in range(1, int(ang_req / step) + 1):
                a = k * step
                R = R_home @ axis_angle(axis, math.radians(sign * a))
                th_new, m = c.spm.ik(R, th)
                if not c.check_pose(R, th_new, m)[0]:
                    break
                th, best = th_new, a
        return best if best is not None and best >= min_deg else None

    def _calibrate(self, tilt_deg, twist_deg):
        c, b = self.ctl, self.board
        try:
            self.cal_status = "moving to neutral"
            R_home = c.hip.R_neutral.copy()
            self._goto(R_home)

            self.cal_status = "measuring gravity and gyro bias (hold still)"
            a0, g0 = b.mean_still(2.0)
            b.est.bias = g0.copy()

            # The gyro-integrated rotations below are only right once the
            # host has measured the sensor's TRUE sample rate (the firmware's
            # timestamps assume 240 Hz; the part really runs ~230.5 Hz, so
            # nominal timing reads every angle ~4% short - HANDOFF 6.1).
            # The fit needs ~2 s of stream, so this normally passes at once.
            self.cal_status = "waiting for the IMU sample-rate estimate"
            t_wait = time.monotonic()
            while not b.clock.converged:
                if time.monotonic() - t_wait > 10.0:
                    raise RuntimeError("IMU sample-rate estimate did not converge (is the stream stalling?)")
                time.sleep(0.1)

            cal = MountCalibration()
            moves = [([1, 0, 0], tilt_deg), ([0, 1, 0], tilt_deg), ([0, 0, 1], twist_deg)]
            used = []
            for axis, ang_req in moves:
                for sign in (+1, -1):
                    # Largest swing (<= requested) whose whole path the safety gate
                    # accepts: a design with a tight joint (v2: leg 1 elbow) would
                    # otherwise refuse the move and the calibration would time out.
                    ang = self._safe_swing(R_home, axis, sign, ang_req)
                    if ang is None:
                        c.log(f"  IMU calibration: no safe swing about effector {'xyz'[axis.index(1)]} "
                              f"{'+' if sign > 0 else '-'}; skipped")
                        continue
                    used.append(sign * ang)
                    R_out = R_home @ axis_angle(axis, math.radians(sign * ang))
                    for R_to in (R_out, R_home):              # out, then back
                        self.cal_status = f"rotating about effector {'xyz'[axis.index(1)]} {sign * ang:+.0f} deg"
                        R_from = self.reference_pose().copy()
                        integ = b.gyro_integrator()
                        self._goto(R_to)
                        dI = integ.stop()
                        R_end = self.reference_pose().copy()
                        dE = rotvec(R_from.T @ R_end)        # what the effector actually did, its own axes
                        cal.add(dE, dI)

            R_EI, info = cal.solve()
            with c.lock:
                R_ref = self.reference_pose().copy()
            up_B = R_ref @ R_EI @ (a0 / np.linalg.norm(a0))
            b.est.R_EI, b.est.up_B, b.est.calibrated = R_EI, up_B, True
            b.est.align(R_ref)
            b.save_calibration()
            info["gravity_in_base"] = up_B.round(4).tolist()
            info["accel_norm_g"] = float(np.linalg.norm(a0))
            info["imu_true_rate_hz"] = round(b.clock.rate_hz, 3)   # rate the gyro was integrated at
            info["swings_deg"] = used                               # x+, x-, y+, y-, z+, z- actually used (safe-limited)
            c.log(f"  calibration swings used (deg, x+ x- y+ y- z+ z-): {used}")
            self.cal_result = info
            self.cal_status = (f"done: residual max {max(info['residual_deg']):.2f} deg, "
                               f"gyro/encoder scale {np.mean(info['gyro_scale']):.4f}, "
                               f"IMU rate {info['imu_true_rate_hz']:.2f} Hz")
            c.log("IMU mount calibration " + self.cal_status)
            c.log(f"  gravity 'up' in base frame = {up_B.round(3).tolist()}  (|a| = {info['accel_norm_g']:.4f} g)")
        except Exception as exc:
            self.cal_status = f"FAILED: {exc}"
            c.log(f"IMU calibration failed: {exc}")
        finally:
            with c.lock:
                c._hold_here()             # leave the rig holding wherever it is, in Pose mode

    # ------------------------------------------------------------------
    # Hooks called by the controller
    # ------------------------------------------------------------------
    def motion_hint(self, still: bool):
        """Tell the estimator whether the rig is being commanded to hold still,
        so gyro bias is only learned when the leg really is not moving."""
        if self.board:
            self.board.est.motion_still_hint = still

    def trim_step(self, R_target: np.ndarray, dt: float) -> np.ndarray:
        """Update and return the trim rotation (base frame, rotation vector)."""
        cfg = self.ctl.cfg.get("imu", {})
        gain = float(cfg.get("trim_gain", 0.0))
        est = self.board.est if self.board else None
        if gain <= 0 or not (self.ok and est.calibrated and est.aligned and self.ctl.armed):
            self.trim *= 0.98                       # bleed off when disabled
            return self.trim
        err = rotvec(R_target @ est.R_BE.T)         # rotation from where the IMU says we are, to target
        up = est.up_B / np.linalg.norm(est.up_B)
        err_tilt = err - (err @ up) * up            # drop the heading component (not observable)
        self.trim += gain * err_tilt * dt
        lim = math.radians(float(cfg.get("trim_limit_deg", 3.0)))
        n = np.linalg.norm(self.trim)
        if n > lim:
            self.trim *= lim / n
        return self.trim

    def snapshot(self, hip) -> dict:
        if not self.board:
            return {"present": False, "cal_status": self.cal_status}
        s = self.board.snapshot()
        s["present"] = True
        s["cal_status"] = self.cal_status
        s["trim_deg"] = np.degrees(self.trim).round(3).tolist()
        est = self.board.est
        if est.calibrated and est.aligned:
            c = self.ctl
            R_imu = est.R_BE
            R_anat = hip.anat_from_effector(R_imu)
            f, a, r = hip.anat_angles(R_anat)
            s["anat"] = {"flex": f, "add": a, "rot": r}
            s["femur"] = hip.femur_points(R_anat)
            # angular velocity in the pelvis frame (anatomical axes)
            w_P = hip.R_PB @ est.w_B
            s["w_pelvis_dps"] = np.degrees(w_P).round(2).tolist()
            s["err_vs_cmd_deg"] = ang_deg(R_imu @ c.R_cmd.T)
            if c.R_meas is not None:
                s["err_vs_enc_deg"] = ang_deg(R_imu @ c.R_meas.T)
                # tilt-only disagreement: compare where each puts gravity
                up = est.up_B
                s["tilt_err_vs_enc_deg"] = math.degrees(math.acos(np.clip(
                    (R_imu.T @ up) @ (c.R_meas.T @ up), -1, 1)))
        return s

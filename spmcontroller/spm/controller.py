"""
controller.py - The real-time loop that turns "desired hip pose" into motor commands.

One background thread runs at `control.loop_hz` (default 50 Hz).  Each tick:

  1. Ask the active motion mode for a target (hip rotation or ring angles).
  2. RATE-LIMIT: move the commanded pose toward the target by no more than
     max_rate * dt.  This makes every transition smooth - switching modes,
     dragging sliders, starting a gait cycle - and caps motor speed.
  3. Inverse kinematics -> ring angles theta (rad).
  4. SAFETY GATE: reachability, distance from singularities, ring-to-ring
     separation (link collisions), max tilt, max ring excursion.  If the new
     pose fails, the previous safe pose is held and the reason is reported.
  5. Ring angles -> motor encoder counts (4:1 gear, direction sign, home offset).
  6. If ARMED: send all three F5 "absolute axis" setpoints in ONE long packet.
  7. Read back one motor's encoder per tick (round-robin) and run forward
     kinematics so the GUI can show the MEASURED pose next to the commanded one.

Mode time only advances while the commanded pose is tracking the mode's
target (error < sync threshold), so time-based motions start cleanly.
"""

from __future__ import annotations

import copy
import json
import math
import threading
import time
import traceback
from pathlib import Path

import numpy as np

from . import protocol as P
from .anatomy import HipMapping
from .bus import BusError, MotorBus, SimPort, list_serial_ports, open_serial
from .kinematics import CoaxialSPM, SPMGeometry, axis_angle, rotvec
from .motion import EffectorMode, GaitMode, JointMode, PlaneMode, PoseMode, parse_gait_csv
from .imu_control import ImuIntegration

COUNTS_PER_RAD = P.ENCODER_COUNTS_PER_REV / (2 * math.pi)


def deep_update(dst: dict, src: dict) -> dict:
    """Recursively merge src into dst (used for partial config updates)."""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst


class Controller:
    def __init__(self, config_path: Path):
        self.config_path = Path(config_path)
        self.cfg = json.loads(self.config_path.read_text())
        self.lock = threading.RLock()            # guards everything below
        self.log_lines: list[str] = []

        # -- connection state --
        self.bus: MotorBus | None = None
        self.sim = False
        self.connected = False
        self.armed = False
        self.homed = False
        self.home_counts = [0, 0, 0]             # encoder counts at the home pose
        # E-stop LATCH: once E-stop is pressed, arming is blocked until the user
        # deliberately resets it (reset_estop).  Resetting only PREPARES a
        # restart - it never moves anything by itself.
        self.estopped = False
        self.ui_seq = 0                          # bumped when the server changes mode params,
                                                 # so the browser re-syncs its sliders
        # Power-loss detection: if a driver misses several encoder reads in a row
        # it may have rebooted, and its multi-turn count restarts at power-up,
        # which would silently corrupt home.  We latch home_suspect and demand a re-home.
        self.fail_streak = [0, 0, 0]
        self.home_suspect = False

        # -- kinematic state --
        self._rebuild_models()
        self.R_cmd = self.hip.R_neutral.copy()   # commanded effector orientation (base frame)
        self.theta_cmd, _ = self.spm.ik(self.R_cmd)
        self.theta_home = self.theta_cmd.copy()
        self.theta_meas = None                   # from encoders
        self.R_meas = None
        self.counts_cmd = [0, 0, 0]
        self.counts_meas = [None, None, None]
        self.safety = {"ok": True, "reason": ""}
        self.metrics = {}
        self.rate_limited = False
        # "View beyond limits" (GUI, for looking only): while ON, poses the
        # safety gate refuses are still SHOWN (if the IK can reach them),
        # but nothing can be sent to the motors.  Three independent guards:
        # it can only be switched on while disarmed, arm() refuses while it
        # is on, and the send step skips the bus while it is on.
        self.view_override = False
        self._safe_R, self._safe_theta = self.R_cmd.copy(), self.theta_cmd.copy()   # last pose the gate passed

        # -- motion modes --
        self.modes = {
            "pose": PoseMode(self.hip),
            "plane": PlaneMode(self.hip),
            "gait": GaitMode(self.hip),
            "joint": JointMode(),
            "effector": EffectorMode(),          # used by the IMU mount calibration
        }
        self.mode = self.modes["pose"]
        self.mode_t = 0.0
        self.running = False                     # timed modes only advance while running

        self._stop = threading.Event()
        self._rr = 0                             # round-robin index for encoder reads
        self.loop_dt_meas = 0.0
        # Femur-mounted IMU board (optional): orientation feedback over USB
        self.cfg.setdefault("imu", {"port": "", "trim_gain": 0.0, "trim_limit_deg": 3.0})
        self.imu = ImuIntegration(self, self.config_path.parent / "imu_cal.json")
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ======================================================================
    # Config / model handling
    # ======================================================================

    def _rebuild_models(self):
        self.geom = SPMGeometry.from_dict(self.cfg["geometry"])
        self.spm = CoaxialSPM(self.geom)
        self.hip = HipMapping(self.cfg)

    def log(self, msg: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"[{stamp}] {msg}")
        del self.log_lines[:-200]
        print(f"[{stamp}] {msg}", flush=True)

    def update_config(self, patch: dict, save: bool = False, keep_femur: bool = False):
        """Merge a partial config from the GUI.  Geometry/mount changes are only
        allowed while DISARMED, because they move the home pose.

        keep_femur (GUI "Remount casing"): if the MOUNT changes but the NEUTRAL
        values do not, recompute neutral so the femur stays attached to the
        femur head exactly as before, i.e. model a real remount of the casing
        on the pelvis.  'mount' and 'neutral' together fix how the femur sits
        on the head: the femur axes in the effector frame are
        F_E = (R_PB @ R_neutral)^T.  Changing the mount alone would silently
        re-bolt the femur at a new angle (seen 2026-11-18: changing only the
        tilt cut flexion REACH from +54 to +33 deg).  Keeping F_E fixed:
        R_neutral_new = R_PB_new^T @ F_E^T."""
        with self.lock:
            touches_geometry = any(k in patch for k in ("geometry", "mount", "neutral", "motors"))
            if touches_geometry and self.armed:
                raise RuntimeError("Disarm before changing geometry, mount or motor settings")

            def changed(sec):
                new, cur = patch.get(sec, {}), self.cfg.get(sec, {})
                return any(not k.startswith("_") and (
                    (isinstance(v, (int, float)) and abs(float(v) - float(cur.get(k, 0))) > 1e-9)
                    or (not isinstance(v, (int, float)) and v != cur.get(k))) for k, v in new.items())

            remount = keep_femur and changed("mount") and not changed("neutral")
            femur_in_head_T = self.hip.R_PB @ self.hip.R_neutral       # = F_E^T, before the change
            deep_update(self.cfg, patch)
            self._rebuild_models()
            if remount:
                Rn = self.hip.R_PB.T @ femur_in_head_T
                # R_neutral = Rz(twist) Ry(tilt_y) Rx(tilt_x)  (anatomy.py) -> Z-Y-X angles
                self.cfg["neutral"].update({
                    "tilt_x_deg": round(math.degrees(math.atan2(Rn[2, 1], Rn[2, 2])), 4),
                    "tilt_y_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, -Rn[2, 0])))), 4),
                    "twist_deg": round(math.degrees(math.atan2(Rn[1, 0], Rn[0, 0])), 4)})
                self._rebuild_models()
                n = self.cfg["neutral"]
                self.log(f"Remount: neutral recomputed to keep the femur on the head "
                         f"(tilt_x {n['tilt_x_deg']:.2f}, tilt_y {n['tilt_y_deg']:.2f}, twist {n['twist_deg']:.2f})")
            for m in self.modes.values():
                if hasattr(m, "hip"):
                    m.hip = self.hip
            if touches_geometry:
                # Home pose changed -> recompute it and reset the command to neutral.
                self.R_cmd = self.hip.R_neutral.copy()
                self.theta_home, _ = self.spm.ik(self.R_cmd)
                self.theta_cmd = self.theta_home.copy()
                if self.homed:
                    self.homed = False
                    self.log("Geometry changed: home invalidated, please set home again")
            if save:
                self.save_config()

    def save_config(self):
        self.config_path.write_text(json.dumps(self.cfg, indent=2))
        self.log(f"Config saved to {self.config_path.name}")

    # ======================================================================
    # Connection / arming
    # ======================================================================

    def connect(self, port: str, baud: int, simulate: bool):
        with self.lock:
            self.disconnect()
            addrs = [m["address"] for m in self.cfg["motors"]]
            if simulate:
                self.bus = MotorBus(SimPort(addrs))
                self.sim = True
                self.log("Connected to SIMULATED drivers")
            else:
                self.bus = MotorBus(open_serial(port, baud))
                self.sim = False
                self.log(f"Opened {port} @ {baud} baud")
            self.connected = True
            self.homed = False
            self.counts_meas = [None, None, None]
            # Probe each driver so the user immediately sees which ones answer.
            for m in self.cfg["motors"]:
                try:
                    c = self.bus.read_encoder(m["address"])
                    self.log(f"  motor addr {m['address']} answered, encoder = {c}")
                except BusError as e:
                    self.log(f"  motor addr {m['address']} NO REPLY ({e})")

    def disconnect(self):
        with self.lock:
            if self.bus:
                if self.armed:
                    self.estop()
                self.bus.close()
            self.bus = None
            self.connected = False
            self.armed = False

    def set_home(self):
        """Declare that the mechanism is physically at the neutral/home pose.

        We record each motor's current encoder count in software instead of
        rewriting the driver's zero, so nothing is stored in the drivers and
        a mistake costs nothing.  Encoder multi-turn counts restart at power-up
        (and the 4:1 gear means one motor turn is only 90 deg of ring), so home
        must be set again after every power cycle."""
        with self.lock:
            if not self.connected:
                raise RuntimeError("Not connected")
            if self.armed:
                raise RuntimeError("Disarm first (motors must be free to be placed at home)")
            counts = [self.bus.read_encoder(m["address"]) for m in self.cfg["motors"]]
            self.home_counts = counts
            self.R_cmd = self.hip.R_neutral.copy()
            self.theta_home, _ = self.spm.ik(self.R_cmd)
            self.theta_cmd = self.theta_home.copy()
            self.counts_cmd = list(counts)
            self.homed = True
            self.home_suspect = False
            self.fail_streak = [0, 0, 0]
            self.log(f"Home set at encoder counts {counts}")

    def set_torque(self, enable: bool):
        """F3 enable/disable on every motor (disable = shafts free to move by hand)."""
        with self.lock:
            if not self.connected:
                raise RuntimeError("Not connected")
            if not enable:
                self.armed = False
            for m in self.cfg["motors"]:
                st = self.bus.status_cmd(P.cmd_enable(m["address"], enable), m["address"])
                self.log(f"  addr {m['address']} torque {'ON' if enable else 'OFF'}: status {st}")

    def arm(self):
        """Start streaming setpoints.  The commanded pose is first snapped to the
        MEASURED pose so arming never causes a jump."""
        with self.lock:
            if not self.connected:
                raise RuntimeError("Not connected")
            if not self.homed:
                raise RuntimeError("Set home first")
            if self.estopped:
                raise RuntimeError("E-stop is latched: clear the hazard, then press 'Reset E-stop'")
            if self.view_override:
                raise RuntimeError("'View beyond limits' is on (viewing only): turn it off before arming")
            if self.home_suspect:
                self.homed = False
                raise RuntimeError("A driver stopped answering (possible power loss): re-home first")
            self._check_no_stall()
            counts = [self.bus.read_encoder(m["address"]) for m in self.cfg["motors"]]
            theta = self._counts_to_theta(counts)
            R, ok = self.spm.fk(theta, self.R_cmd)
            if not ok:
                raise RuntimeError("Forward kinematics failed on the measured angles; re-home")
            self.R_cmd, self.theta_cmd = R, theta
            self._hold_here()                    # targets := measured pose -> nothing moves on arm
            self.set_torque(True)
            self.armed = True
            self.log("ARMED: streaming setpoints to motors")

    def set_view_override(self, on: bool):
        """Show poses beyond the safety limits in the 3D view (VIEWING ONLY).
        Allowed only while disarmed; arming is refused while it is on."""
        with self.lock:
            on = bool(on)
            if on == self.view_override:
                return
            if on:
                if self.armed:
                    raise RuntimeError("Disarm first: limits can only be overridden for viewing, never while armed")
                self.view_override = True
                self.log("VIEW BEYOND LIMITS on: refused poses are shown but nothing can be sent (ARM is blocked)")
                return
            self.view_override = False
            # Leave the view at a pose the gate accepts: keep the current one
            # if it passes, otherwise go back to the last safe one.
            th, m = self.spm.ik(self.R_cmd, self.theta_cmd)
            if not self.check_pose(self.R_cmd, th, m)[0]:
                self.R_cmd, self.theta_cmd = self._safe_R.copy(), self._safe_theta.copy()
            self.safety = {"ok": True, "reason": ""}
            self.log("VIEW BEYOND LIMITS off: view back inside the limits")

    def disarm(self):
        with self.lock:
            self.armed = False
            self.running = False
            self.log("Disarmed (motors hold their last position)")

    def estop(self):
        """Broadcast F7 to every driver (one frame, no reply) and disarm."""
        with self.lock:
            self.armed = False
            self.running = False
            self.estopped = True                     # latched until reset_estop()
            if self.bus:
                for _ in range(2):                   # send twice in case of a glitch
                    self.bus.send(P.cmd_estop(P.BROADCAST))
            self.log("EMERGENCY STOP sent (broadcast F7)")

    def _check_no_stall(self):
        """Read 0x3E (shaft protection state) on every motor.  A stalled driver
        has released its shaft, so it must be cleared (0x3D) before arming."""
        stalled = []
        for m in self.cfg["motors"]:
            a = m["address"]
            if self.bus.status_cmd(P.cmd_read(a, P.CMD_READ_STALL), a) == 1:
                stalled.append(a)
        if stalled:
            raise RuntimeError(f"Stall protection active on motor addr {stalled}: "
                               "check for a jam, then 'Release stalls'")

    def release_stalls(self):
        """Send 0x3D (release locked-rotor protection) to every motor."""
        with self.lock:
            if not self.connected:
                raise RuntimeError("Not connected")
            for m in self.cfg["motors"]:
                a = m["address"]
                st = self.bus.status_cmd(P.cmd_release_stall(a), a)
                self.log(f"  addr {a} release stall: {'OK' if st == 1 else f'status {st}'}")

    def _hold_here(self):
        """Make every motion mode's target equal to the CURRENT commanded pose,
        stop timed modes, and select Pose mode.  Used on arm/E-stop reset so the
        leg stays exactly where it is until the operator asks for motion."""
        f, a, r = self.hip.anat_angles(self.hip.anat_from_effector(self.R_cmd))
        pose = self.modes["pose"]
        pose.flex, pose.add, pose.rot = round(f, 1), round(a, 1), round(r, 1)
        self.modes["joint"].offsets_deg = [round(float(x), 1) for x in
                                           np.degrees(self.theta_cmd - self.theta_home)]
        self.mode = pose
        self.mode_t = 0.0
        self.running = False
        self.ui_seq += 1

    def reset_estop(self):
        """Deliberate operator reset after an E-stop (does NOT restart motion).

        Checks, in order:
          1. the bus answers every motor (catches a power cut / unplugged cable),
          2. no motor is in stall protection,
          3. the measured encoder pose is consistent with the stored home -
             if the drivers were power-cycled their counts restart, so the
             pose will be implausible and you must re-home instead.
        Then sets every mode's target to the measured pose and clears the latch.
        You still have to press ARM to move again."""
        with self.lock:
            if not self.estopped:
                return
            if self.connected:
                try:
                    counts = [self.bus.read_encoder(m["address"]) for m in self.cfg["motors"]]
                except Exception as exc:
                    raise RuntimeError(f"Driver not answering ({exc}). Check power/wiring, "
                                       "reconnect, then reset again") from exc
                self._check_no_stall()
                if self.homed and self.home_suspect:
                    self.homed = False
                    self.log("A driver dropped off the bus during the stop (possible power loss): "
                             "home cleared - Torque OFF, place at neutral, Set home")
                if self.homed:
                    # How far did each ring end up from where it was last commanded?
                    drift = [abs(c - cc) / (abs(self._gear(i)) * COUNTS_PER_RAD)
                             for i, (c, cc) in enumerate(zip(counts, self.counts_cmd))]
                    self.log("Ring drift since stop (deg): " + ", ".join(f"{np.degrees(d):.2f}" for d in drift))
                    theta = self._counts_to_theta(counts)
                    R, ok = self.spm.fk(theta, self.R_cmd)
                    _, margin = self.spm.ik(R) if ok else (None, np.array([-1.0]))
                    good, why, _ = self.check_pose(R, theta, margin) if ok else (False, "FK failed", {})
                    if not good:
                        self.homed = False
                        self.log(f"Measured pose implausible after E-stop ({why}): "
                                 "home cleared - Torque OFF, place at neutral, Set home")
                    else:
                        self.R_cmd, self.theta_cmd = R, theta
                        self.R_meas, self.theta_meas = R, theta
            self._hold_here()
            self.estopped = False
            self.log("E-stop reset: targets set to the current pose. Press ARM to resume.")

    # ======================================================================
    # Modes
    # ======================================================================

    def set_mode(self, name: str, params: dict):
        with self.lock:
            m = self.modes[name]
            for k, v in params.items():
                if k == "csv":
                    m.set_keys(parse_gait_csv(v["text"]), v.get("name", "csv"))
                    self.log(f"Loaded gait CSV '{m.label}'")
                elif k == "reset_keys":
                    from .motion import DEFAULT_GAIT_KEYS
                    m.set_keys(DEFAULT_GAIT_KEYS, "built-in")
                elif hasattr(m, k):
                    setattr(m, k, tuple(v) if k in ("base", "scale") else v)
            if m is not self.mode:
                self.mode = m
                self.mode_t = 0.0
                self.running = False

    def set_running(self, run: bool):
        with self.lock:
            self.running = run
            if not run:
                pass                                     # keep mode_t so resume continues

    def reset_time(self):
        with self.lock:
            self.mode_t = 0.0

    # ======================================================================
    # Unit conversions
    # ======================================================================

    def _gear(self, i):
        m = self.cfg["motors"][i]
        return self.cfg["control"]["gear_ratio"] * (1 if m.get("direction", 1) >= 0 else -1)

    def _theta_to_counts(self, theta):
        return [int(round(self.home_counts[i] + self._gear(i) * (theta[i] - self.theta_home[i]) * COUNTS_PER_RAD))
                for i in range(3)]

    def _counts_to_theta(self, counts):
        return np.array([self.theta_home[i] + (counts[i] - self.home_counts[i]) / (self._gear(i) * COUNTS_PER_RAD)
                         for i in range(3)])

    # ======================================================================
    # Safety
    # ======================================================================

    def check_pose(self, R, theta, margin) -> tuple[bool, str, dict]:
        lim = self.cfg["limits"]
        if not np.all(np.isfinite(theta)):
            bad = [i + 1 for i in range(3) if not np.isfinite(theta[i])]
            return False, f"unreachable (leg {bad})", {}
        met = self.spm.singularity_metrics(R, theta)
        met["leg_margin"] = float(np.min(margin))
        tilt = math.degrees(math.acos(np.clip((R @ np.array([0, 0, 1.0]))[2], -1, 1)))
        met["tilt_deg"] = tilt
        # pairwise ring separation (deg, 0..180)
        seps = []
        for i in range(3):
            for j in range(i + 1, 3):
                d = abs((theta[i] - theta[j] + math.pi) % (2 * math.pi) - math.pi)
                seps.append(math.degrees(d))
        met["min_ring_sep_deg"] = min(seps)
        excursion = np.degrees(np.abs(theta - self.theta_home))
        met["max_excursion_deg"] = float(np.max(excursion))
        # Passive-joint fold limits (link collisions measured in the CAD, see
        # experiments/cad_collision.py).  Angles are relative to the pose the
        # CAD assembly was saved in (joint_limits.ref_R_BE).  That is NOT
        # necessarily anatomical neutral: after remounting the base, neutral
        # moves but the link geometry, and so these limits, do not.
        # (Older configs without ref_R_BE: neutral was the CAD pose.)
        jl = lim.get("joint_limits")
        joint_bad = None
        if jl:
            R_ref = np.array(jl["ref_R_BE"], float) if "ref_R_BE" in jl else self.hip.R_neutral
            th_ref, _ = self.spm.ik(R_ref)
            elbow, wrist = self.spm.joint_fold_angles(R, theta, R_ref, th_ref)
            margin = float(jl.get("margin_deg", 0.0))
            worst = math.inf
            for kind, angles in (("elbow", elbow), ("wrist", wrist)):
                for i, a in enumerate(angles):
                    lo, hi = jl[f"{kind}_deg"][i]
                    # a bound at +-180 means "no collision that way"
                    room = min(a - lo if lo > -180 else math.inf, hi - a if hi < 180 else math.inf) - margin
                    if room < worst:
                        worst = room
                    if room < 0 and joint_bad is None:
                        joint_bad = f"leg {i + 1} {kind} at {a:+.1f} deg, limit {lo:+.0f}..{hi:+.0f} (link collision)"
            met["joint_room_deg"] = worst if math.isfinite(worst) else 180.0
        if met["leg_margin"] < lim["min_leg_margin"]:
            return False, "leg near full stretch (serial singularity)", met
        if met["parallel_index"] < lim["min_parallel_index"]:
            return False, "near parallel singularity", met
        if met["serial_index"] < lim["min_serial_index"]:
            return False, "near serial singularity", met
        if tilt > lim["max_tilt_deg"]:
            return False, f"tilt {tilt:.1f} > {lim['max_tilt_deg']} deg", met
        if met["min_ring_sep_deg"] < lim["min_ring_separation_deg"]:
            return False, "rings too close (link collision)", met
        if joint_bad:
            return False, joint_bad, met
        if met["max_excursion_deg"] > lim["max_ring_excursion_deg"]:
            return False, "ring excursion limit", met
        return True, "", met

    # ======================================================================
    # The loop
    # ======================================================================

    def _loop(self):
        next_t = time.perf_counter()
        last = next_t
        while not self._stop.is_set():
            hz = max(5.0, float(self.cfg["control"]["loop_hz"]))
            dt = 1.0 / hz
            now = time.perf_counter()
            self.loop_dt_meas = now - last
            last = now
            try:
                with self.lock:
                    self._tick(dt)
            except Exception as exc:                         # never let the thread die
                self.log(f"control loop error: {exc}")
                traceback.print_exc()
                if self.armed:
                    self.estop()
            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()                 # overran; resync

    def _tick(self, dt):
        ctl = self.cfg["control"]
        max_rate = math.radians(ctl["max_rate_deg_s"])

        # While E-stopped, FREEZE the command: do not chase the mode target
        # (nothing is sent anyway, but the 3D view would otherwise show a pose
        # the leg never reached).  Track the measured pose instead, then only
        # run feedback.
        if self.estopped:
            if self.R_meas is not None and self.theta_meas is not None:
                self.R_cmd, self.theta_cmd = self.R_meas.copy(), self.theta_meas.copy()
            self.rate_limited = False
            self._feedback(ctl)
            return

        # ---- 1. target from the mode -----------------------------------
        kind, tgt = self.mode.target(self.mode_t)

        # ---- 2. rate-limited step toward the target ---------------------
        if kind in ("anat", "effector"):
            R_target = self.hip.effector_from_anat(tgt) if kind == "anat" else tgt
            err = rotvec(R_target @ self.R_cmd.T)           # rotation still to go
            err_ang = float(np.linalg.norm(err))
            step = min(err_ang, max_rate * dt)
            self.rate_limited = err_ang > max_rate * dt + 1e-9
            R_new = axis_angle(err, step) @ self.R_cmd if err_ang > 1e-12 else R_target
            theta_new, margin = self.spm.ik(R_new, self.theta_cmd)
        else:                                               # joint mode
            theta_target = self.theta_home + tgt
            d = theta_target - self.theta_cmd
            # joint rate cap in ring degrees/s (same number as the pose cap)
            scale = min(1.0, (max_rate * dt) / max(1e-12, float(np.max(np.abs(d)))))
            self.rate_limited = scale < 1.0
            err_ang = float(np.max(np.abs(d)))
            theta_new = self.theta_cmd + d * scale
            R_new, ok = self.spm.fk(theta_new, self.R_cmd)
            _, margin = self.spm.ik(R_new)
            if not ok:
                theta_new = np.full(3, np.nan)

        # ---- 3/4. safety gate ------------------------------------------
        ok, reason, met = self.check_pose(R_new, theta_new, margin)
        if met:
            self.metrics = met
        # In view-only override a refused pose is still SHOWN, as long as the
        # IK could solve it (an unreachable pose cannot be drawn).  The send
        # step below never transmits while the override is on.
        show_anyway = (not ok and self.view_override and not self.armed
                       and bool(np.all(np.isfinite(theta_new))))
        accepted = ok or show_anyway
        if ok:
            self.R_cmd, self.theta_cmd = R_new, theta_new
            self._safe_R, self._safe_theta = R_new.copy(), theta_new.copy()
            self.safety = {"ok": True, "reason": ""}
        elif show_anyway:
            self.R_cmd, self.theta_cmd = R_new, theta_new
            self.safety = {"ok": False, "reason": reason, "overridden": True}
        else:
            if self.safety.get("reason") != reason:
                self.log(f"Pose rejected: {reason} - holding last safe pose")
            # 'unreachable' = no ring angles exist for this pose (no IK solution):
            # even the view-only override cannot draw it.  The GUI says so.
            self.safety = {"ok": False, "reason": reason,
                           "unreachable": not bool(np.all(np.isfinite(theta_new)))}

        # ---- advance mode time only while tracking ---------------------
        sync = math.radians(ctl["sync_threshold_deg"])
        if self.mode.timed and self.running and accepted and (err_ang <= sync or not self.rate_limited):
            self.mode_t += dt

        # ---- IMU: tell it whether we are holding still; optional tilt trim --
        self.imu.motion_hint(not self.rate_limited and not (self.mode.timed and self.running) and err_ang < 1e-3)
        theta_send = self.theta_cmd
        if kind in ("anat", "effector") and self.imu.board is not None:
            trim = self.imu.trim_step(R_target, dt)
            if np.linalg.norm(trim) > 1e-6:
                R_send = axis_angle(trim, float(np.linalg.norm(trim))) @ self.R_cmd
                th, mg = self.spm.ik(R_send, self.theta_cmd)
                if self.check_pose(R_send, th, mg)[0]:
                    theta_send = th                     # trimmed pose is safe -> use it

        # ---- 5/6. send setpoints ------------------------------------------
        new_counts = self._theta_to_counts(theta_send) if self.homed else self.counts_cmd
        if self.armed and self.bus and not self.view_override:   # override = viewing only, never sent
            targets = []
            for i, m in enumerate(self.cfg["motors"]):
                delta = abs(new_counts[i] - self.counts_cmd[i])
                # speed needed to cover this tick's motion in one tick, with headroom
                rpm = delta / dt * 60.0 / P.ENCODER_COUNTS_PER_REV
                rpm = min(ctl["max_motor_rpm"], max(ctl["min_motor_rpm"], math.ceil(rpm * ctl["speed_headroom"])))
                targets.append((m["address"], rpm, ctl["acc"], new_counts[i]))
            self.bus.send(P.long_abs_axis(targets))
        self.counts_cmd = new_counts

        self._feedback(ctl)

    def _feedback(self, ctl):
        # ---- 7. feedback (one encoder per tick) ----------------------------
        if self.connected and self.bus and ctl.get("read_feedback", True):
            i = self._rr % 3
            self._rr += 1
            try:
                self.counts_meas[i] = self.bus.read_encoder(self.cfg["motors"][i]["address"])
                self.fail_streak[i] = 0
            except BusError:
                self.counts_meas[i] = None
                self.fail_streak[i] += 1
                if self.fail_streak[i] == 3 and self.homed and not self.home_suspect:
                    self.home_suspect = True
                    addr = self.cfg["motors"][i]["address"]
                    self.log(f"Motor addr {addr} stopped answering (power loss / cable?)")
                    if self.armed:
                        self.estop()                     # fault while moving -> latched stop
            if self.homed and all(c is not None for c in self.counts_meas):
                th = self._counts_to_theta(self.counts_meas)
                R, good = self.spm.fk(th, self.R_meas if self.R_meas is not None else self.R_cmd)
                self.theta_meas, self.R_meas = th, (R if good else None)

    # ======================================================================
    # Snapshot for the GUI
    # ======================================================================

    def snapshot(self) -> dict:
        with self.lock:
            R_anat_cmd = self.hip.anat_from_effector(self.R_cmd)
            flex, add, rot = self.hip.anat_angles(R_anat_cmd)
            vis = self.cfg["visual"]
            snap = {
                "connected": self.connected, "sim": self.sim, "armed": self.armed,
                "view_override": self.view_override,
                "homed": self.homed, "running": self.running,
                "estopped": self.estopped, "ui_seq": self.ui_seq, "home_suspect": self.home_suspect,
                "mode": self.mode.name, "mode_params": self.mode.describe(), "mode_t": self.mode_t,
                "anat_cmd": {"flex": flex, "add": add, "rot": rot},
                "theta_cmd_deg": np.degrees(self.theta_cmd).tolist(),
                "theta_rel_deg": np.degrees(self.theta_cmd - self.theta_home).tolist(),
                "counts_cmd": self.counts_cmd, "counts_meas": self.counts_meas,
                "safety": self.safety, "metrics": self.metrics, "rate_limited": self.rate_limited,
                "loop_ms": self.loop_dt_meas * 1000,
                "bus": {"tx": self.bus.tx_frames, "rx_err": self.bus.rx_errors} if self.bus else None,
                "R_PB": self.hip.R_PB.tolist(),
                "linkage_cmd": self.spm.drawing(self.R_cmd, self.theta_cmd, vis["r_prox_m"], vis["r_dist_m"]),
                "femur_cmd": self.hip.femur_points(R_anat_cmd),
                "log": self.log_lines[-30:],
                "imu": self.imu.snapshot(self.hip),
            }
            if self.mode.name == "gait":
                snap["gait_phase"] = self.mode.phase(self.mode_t)
            if self.R_meas is not None and self.theta_meas is not None:
                R_anat_meas = self.hip.anat_from_effector(self.R_meas)
                f2, a2, r2 = self.hip.anat_angles(R_anat_meas)
                snap["anat_meas"] = {"flex": f2, "add": a2, "rot": r2}
                snap["theta_meas_deg"] = np.degrees(self.theta_meas).tolist()
                snap["femur_meas"] = self.hip.femur_points(R_anat_meas)
            return snap

    def gait_preview(self) -> dict:
        """The gait mode's sampled curves (for the GUI plot), plus whether the
        safety gate would accept each sample: 'ok' (bool per sample) and
        'reason' (why not, '' if ok).  The GUI shades the refused phases, so
        the Gait tab shows where the cycle goes beyond the limits before
        anything moves.  IK is chained sample to sample from home, like the
        control loop, so the ring angles stay on one continuous branch."""
        with self.lock:
            pv = self.modes["gait"].preview()
            ok, reason = [], []
            th = self.theta_home.copy()
            for f, a, r in zip(pv["flex"], pv["add"], pv["rot"]):
                R = self.hip.effector_from_anat(self.hip.anat_matrix(f, a, r))
                th_new, m = self.spm.ik(R, th)
                good, why, _ = self.check_pose(R, th_new, m)
                ok.append(bool(good))
                reason.append("" if good else why)
                if np.all(np.isfinite(th_new)):
                    th = th_new
            pv["ok"], pv["reason"] = ok, reason
            return pv

    def static_info(self) -> dict:
        with self.lock:
            return {"config": copy.deepcopy(self.cfg), "ports": list_serial_ports(),
                    "gait_preview": self.gait_preview(),
                    "theta_home_deg": np.degrees(self.theta_home).tolist()}

    def shutdown(self):
        self._stop.set()
        self.imu.disconnect()
        self.disconnect()

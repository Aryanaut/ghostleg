"""
Unit tests.  Run with either:
    python -m pytest tests            (if pytest is installed)
    python tests/test_all.py          (plain Python, no extra packages)

The protocol tests use the exact example frames printed in the MKS
SERVO42E/57E RS485 manual V1.0.1, so they prove our framing/checksum matches
what the drivers expect.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spm import protocol as P                                     # noqa: E402
from spm.anatomy import HipMapping                                # noqa: E402
from spm.bus import MotorBus, SimPort                             # noqa: E402
from spm.kinematics import CoaxialSPM, SPMGeometry, axis_angle, rotvec   # noqa: E402
from spm.motion import PeriodicCurve, parse_gait_csv              # noqa: E402

H = bytes.fromhex


# ---------------------------------------------------------------- protocol --

def test_checksum_manual_example():
    # Part 3: FA 01 80 00 -> CRC 0x7B
    assert P.checksum(H("FA018000")) == 0x7B


def test_read_encoder_frame():
    # Part 6.3: send "FA 01 30 2B"
    assert P.cmd_read(1, P.CMD_READ_ENCODER_CARRY) == H("FA01302B")


def test_abs_axis_frames_manual():
    # Part 5.9 examples (position mode 4)
    assert P.cmd_abs_axis(1, 600, 2, 0x4000) == H("FA01F50258020000400" "08C")
    assert P.cmd_abs_axis(1, 600, 2, -0x4000) == H("FA01F5025802FFFFC0000A")
    # Part 5.9 stop examples
    assert P.cmd_abs_axis_stop(1, 4) == H("FA01F5000004000000" "00F4")
    assert P.cmd_abs_axis_stop(1, 0) == H("FA01F5000000000000" "00F0")


def test_long_packet_manual_example():
    # Part 4.10: the five-motor long packet ends with checksum 0x11
    cmds = [
        (1, 0xF6, H("00320A")), (2, 0xF6, H("806420")),
        (3, 0xFD, H("012C02000004E2")), (4, 0xF4, H("0258640019" "00")),
        (5, 0xF5, H("04B0C8000C8000")),
    ]
    pkt = P.long_packet(cmds)
    assert len(pkt) == 52 and pkt[0] == 0xFC
    assert pkt[-1] == 0x11


def test_long_packet_padding_is_duplicate_of_first():
    pkt = P.long_abs_axis([(1, 10, 250, 100), (2, 10, 250, 200), (3, 10, 250, 300)])
    slot = lambda i: pkt[1 + 10 * i: 11 + 10 * i]
    assert slot(3) == slot(0) and slot(4) == slot(0)


def test_parse_reply():
    raw = H("FB01F501")
    raw += bytes([P.checksum(raw)])
    assert P.parse_reply(raw, 1, 0xF5) == b"\x01"


def test_int48_decode():
    assert P.decode_int48(H("FFFFFFFFFFF0")) == -16
    assert P.decode_int48(H("000000007FF0")) == 0x7FF0


# -------------------------------------------------------------- simulator --

def test_sim_bus_roundtrip():
    bus = MotorBus(SimPort((1, 2, 3)))
    assert bus.read_encoder(2) == 0
    assert bus.status_cmd(P.cmd_enable(1, True), 1) == 1


# ------------------------------------------------------------- kinematics --

def _spm(a1=50):
    return CoaxialSPM(SPMGeometry(alpha1_deg=[a1] * 3, alpha2_deg=[90] * 3))


def test_home_is_symmetric():
    th, m = _spm().ik(np.eye(3))
    d = np.degrees(th)
    assert np.allclose(sorted(((d - d[0]) % 360)), [0, 120, 240], atol=1e-6)
    assert np.all(m > 0.99)


def test_ik_fk_roundtrip():
    k = _spm()
    rng = np.random.default_rng(1)
    th0, _ = k.ik(np.eye(3))
    for _ in range(200):
        R = axis_angle(rng.normal(size=3), math.radians(rng.uniform(0, 30)))
        th, m = k.ik(R, th0)
        assert np.all(m > 0)
        R2, ok = k.fk(th, np.eye(3))
        assert ok
        assert np.linalg.norm(rotvec(R2 @ R.T)) < 1e-7


def test_pure_twist_moves_all_rings_equally():
    # Coaxial SPM: rotating the effector about the common axis turns all three rings together.
    k = _spm()
    th0, _ = k.ik(np.eye(3))
    th1, _ = k.ik(axis_angle([0, 0, 1], math.radians(30)), th0)
    assert np.allclose(np.degrees(th1 - th0), 30, atol=1e-6)


# ---------------------------------------------------------------- anatomy --

def _hip(side="right"):
    return HipMapping({"mount": {"side": side, "tilt_down_deg": 45, "forward_deg": 20}})


def test_anat_angles_roundtrip():
    for side in ("right", "left"):
        hip = _hip(side)
        for f, a, r in [(30, 0, 0), (-10, 8, -5), (20, -5, 7)]:
            got = hip.anat_angles(hip.anat_matrix(f, a, r))
            assert np.allclose(got, (f, a, r), atol=1e-9)


def test_flexion_moves_knee_forward_and_adduction_medial():
    hip = _hip("right")
    knee0 = hip.femur0["knee"]
    assert (hip.anat_matrix(30, 0, 0) @ knee0)[0] > 0          # anterior
    assert (hip.anat_matrix(0, 10, 0) @ knee0)[2] < 0          # right leg: medial is -Z
    hipL = _hip("left")
    assert (hipL.anat_matrix(0, 10, 0) @ hipL.femur0["knee"])[2] > 0   # left leg: medial is +Z


def test_effector_mapping_roundtrip():
    hip = _hip()
    R_anat = hip.anat_matrix(25, 5, -3)
    assert np.allclose(hip.anat_from_effector(hip.effector_from_anat(R_anat)), R_anat)


# ----------------------------------------------------------------- motion --

def test_periodic_curve_hits_keys_and_wraps():
    c = PeriodicCurve([(0, 30), (50, -10), (80, 30)])
    assert abs(c(0) - 30) < 1e-9 and abs(c(50) + 10) < 1e-9
    assert abs(c(99.999) - c(0)) < 0.01


def test_csv_parse():
    text = "phase,flex,add,rot\n0,30,0,0\n50,-10,5,2\n80,20,-3,-1\n100,30,0,0\n"
    keys = parse_gait_csv(text)
    assert len(keys["flex"]) == 3 and keys["flex"][1] == (50.0, -10.0)


# ------------------------------------------------------- E-stop recovery --

def _armed_controller(tmp_cfg_dir=None):
    import json, shutil, tempfile, time
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(Path(__file__).resolve().parents[1] / "config.json", d / "config.json")
    c = Controller(d / "config.json")
    c.connect("", 0, simulate=True)
    c.set_home()
    c.arm()
    c.set_mode("pose", {"flex": 20, "add": 0, "rot": 0})
    time.sleep(0.12)                         # stop part-way through the move
    return c


def test_estop_latches_and_reset_holds_pose():
    import time
    c = _armed_controller()
    try:
        c.estop()
        assert c.estopped and not c.armed
        try:
            c.arm(); assert False, "arm must be refused while latched"
        except RuntimeError:
            pass
        time.sleep(0.4)
        snap = c.snapshot()
        # while latched the command must track the measured pose, not the slider target
        assert abs(snap["anat_cmd"]["flex"] - snap["anat_meas"]["flex"]) < 0.5
        flex_at_stop = snap["anat_cmd"]["flex"]
        c.reset_estop()
        assert not c.estopped and c.homed
        # targets now equal the current pose -> arming must not move the leg
        c.arm()
        time.sleep(0.5)
        assert abs(c.snapshot()["anat_cmd"]["flex"] - flex_at_stop) < 0.2
        assert c.modes["pose"].flex == round(flex_at_stop, 1)
    finally:
        c.shutdown()


def test_reset_refused_while_stalled_then_release():
    c = _armed_controller()
    try:
        c.estop()
        c.bus.port.motors[2].stalled = True
        try:
            c.reset_estop(); assert False, "reset must fail while a motor is stalled"
        except RuntimeError as e:
            assert "Stall" in str(e)
        c.release_stalls()
        c.reset_estop()
        assert not c.estopped
    finally:
        c.shutdown()


def test_power_loss_forces_rehome():
    import time
    c = _armed_controller()
    try:
        m = c.bus.port.motors[3]
        m.powered = False                    # driver loses power while armed
        time.sleep(0.4)                      # >= 3 missed reads -> auto E-stop
        assert c.estopped and c.home_suspect and not c.armed
        m.power_cycle_restore()              # power comes back, count restarted
        c.reset_estop()
        assert not c.homed                   # home must be redone
        try:
            c.arm(); assert False
        except RuntimeError:
            pass
    finally:
        c.shutdown()


# --------------------------------------------------------------- IMU board --

def test_usb_frame_parser_resyncs_and_passes_text():
    import struct
    from spm.imu_board import UsbFrameParser, build_frame, IMU_FMT, T_IMU, T_TEXT
    imu = struct.pack(IMU_FMT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 7)
    bad = bytearray(build_frame(T_IMU, imu)); bad[9] ^= 0x55
    data = b"boot junk\r\n" + build_frame(T_TEXT, b"hi\n") + bytes(bad) + build_frame(T_IMU, imu)
    p = UsbFrameParser(); got = []
    for i in range(0, len(data), 5):
        got += p.feed(data[i:i + 5])
    assert [g[0] for g in got] == [T_TEXT, T_IMU] and p.crc_errors == 1
    assert bytes(p.loose_text).startswith(b"boot junk")


def test_crc_matches_firmware_definition():
    from spm.imu_board import crc16_ccitt_false
    assert crc16_ccitt_false(b"123456789") == 0x29B1        # CRC-16/CCITT-FALSE check value


def test_mount_calibration_recovers_rotation():
    from spm.imu_board import MountCalibration
    rng = np.random.default_rng(5)
    R = axis_angle(rng.normal(size=3), 2.1)
    cal = MountCalibration()
    for ax in np.eye(3):
        for sgn in (1, -1):
            dE = ax * sgn * math.radians(15)
            cal.add(dE, R.T @ dE + rng.normal(0, 1e-4, 3))
    Rs, info = cal.solve()
    assert np.linalg.norm(rotvec(Rs @ R.T)) < math.radians(0.05)


# config_v1_cad.json = the v1 config (measured from 'bone-in SPM.step' on 2026-11-17).
# Tests that check FACTS of that CAD use it, so they keep testing the code when
# config.json moves to a newer design (v2 active since 2026-11-18).
V1_CFG = Path(__file__).resolve().parent / "config_v1_cad.json"


def test_config_reproduces_cad_pose():
    """The v1 geometry/mount/neutral were measured from 'bone-in SPM.step'
    (experiments/cad_axes.py).  At anatomical neutral the IK must give the ring
    angles the CAD assembly shows, and the femur must hang straight down."""
    import json
    from spm.anatomy import HipMapping
    from spm.kinematics import CoaxialSPM, SPMGeometry
    cfg = json.loads(V1_CFG.read_text())
    hip = HipMapping(cfg)
    spm = CoaxialSPM(SPMGeometry.from_dict(cfg["geometry"]))
    th, margin = spm.ik(hip.R_neutral)
    cad = np.radians([80.820, -154.198, -3.197])            # ring azimuths of the three middle joints in the CAD
    err = np.degrees(np.angle(np.exp(1j * (th - cad))))
    assert np.abs(err).max() < 0.01, err
    assert np.all(margin > 0.5)                             # comfortably inside the workspace at neutral
    f, a, r = hip.anat_angles(hip.anat_from_effector(hip.R_neutral))
    assert max(abs(f), abs(a), abs(r)) < 1e-6


def test_joint_fold_angles_match_link_rotations():
    """elbow_i must be the rotation of middle arm i relative to arm i about w_i
    (and 0 at the reference pose), wrist_i the rotation of middle arm i
    relative to the femur head about v_i."""
    import json
    from spm.anatomy import HipMapping
    from spm.kinematics import CoaxialSPM, SPMGeometry
    cfg = json.loads((Path(__file__).resolve().parents[1] / "config.json").read_text())
    hip = HipMapping(cfg)
    spm = CoaxialSPM(SPMGeometry.from_dict(cfg["geometry"]))
    R0 = hip.R_neutral
    th0, _ = spm.ik(R0)
    e, w = spm.joint_fold_angles(R0, th0, R0, th0)
    assert max(map(abs, e + w)) < 1e-9
    R = hip.effector_from_anat(hip.anat_matrix(25, 8, -6))
    th, _ = spm.ik(R, th0)
    elbow, wrist = spm.joint_fold_angles(R, th, R0, th0)
    W0, V0, W, V = spm.w_world(th0), spm.v_world(R0), spm.w_world(th), spm.v_world(R)
    for i in range(3):
        # middle-arm rotation = frame (w, v, w x v) now vs at the reference
        F0 = np.column_stack([W0[i], V0[i], np.cross(W0[i], V0[i])])
        F1 = np.column_stack([W[i], V[i], np.cross(W[i], V[i])])
        Q_mid = F1 @ F0.T
        dz = th[i] - th0[i]
        Q_arm = np.array([[math.cos(dz), -math.sin(dz), 0], [math.sin(dz), math.cos(dz), 0], [0, 0, 1]])
        rel = rotvec(Q_arm.T @ Q_mid)                       # middle arm seen from the arm: a rotation about w0
        assert np.allclose(rel, W0[i] * math.radians(elbow[i]), atol=1e-9)
        rel = rotvec((R @ R0.T).T @ Q_mid)                  # middle arm seen from the head: a rotation about v0
        assert np.allclose(rel, V0[i] * math.radians(wrist[i]), atol=1e-9)


def test_safety_gate_refuses_elbow_fold_collision():
    """With the CAD joint limits in config.json, a pose that folds an elbow
    into its arm is refused even though every other check passes."""
    import json, shutil, tempfile
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(Path(__file__).resolve().parents[1] / "config.json", d / "config.json")
    c = Controller(d / "config.json")
    try:
        assert "joint_limits" in c.cfg["limits"]
        lim = c.cfg["limits"]
        found = None
        for f in range(-30, 101, 5):
            for a in range(-40, 41, 5):
                for r in range(-50, 51, 10):
                    R = c.hip.effector_from_anat(c.hip.anat_matrix(f, a, r))
                    th, m = c.spm.ik(R, c.theta_home)
                    ok, why, met = c.check_pose(R, th, m)
                    if not ok and "elbow" in why:
                        found = (f, a, r, why)
                        break
                if found: break
            if found: break
        assert found, "no elbow-limited pose found"
        # and the same pose passes once the joint limits are removed (so it really was the elbow)
        jl = lim.pop("joint_limits")
        R = c.hip.effector_from_anat(c.hip.anat_matrix(*found[:3]))
        th, m = c.spm.ik(R, c.theta_home)
        assert c.check_pose(R, th, m)[0]
        lim["joint_limits"] = jl
        # neutral itself has room on every joint
        th, m = c.spm.ik(c.hip.R_neutral, c.theta_home)
        ok, why, met = c.check_pose(c.hip.R_neutral, th, m)
        assert ok and met["joint_room_deg"] > 0
    finally:
        c.shutdown()


def test_joint_limits_follow_cad_pose_not_neutral():
    """Remounting the base changes neutral but not the link geometry: the
    elbow/wrist limits must stay referenced to the CAD pose (ref_R_BE)."""
    import json, shutil, tempfile
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    src = json.loads((Path(__file__).resolve().parents[1] / "config.json").read_text())
    R_cad = np.array(src["limits"]["joint_limits"]["ref_R_BE"])
    rooms = []
    for neutral in (src["neutral"], {"tilt_x_deg": -10, "tilt_y_deg": 5, "twist_deg": 0}):
        cfg = json.loads(json.dumps(src))
        cfg["neutral"].update(neutral)
        (d / "config.json").write_text(json.dumps(cfg))
        c = Controller(d / "config.json")
        try:
            th, m = c.spm.ik(R_cad, c.theta_home)
            ok, why, met = c.check_pose(R_cad, th, m)
            rooms.append(met["joint_room_deg"])
        finally:
            c.shutdown()
    assert abs(rooms[0] - rooms[1]) < 1e-6 and rooms[0] > 10, rooms


def test_view_override_shows_but_never_sends():
    """'View beyond limits': refused poses are shown, but the override can't be
    switched on while armed, ARM is refused while it is on, nothing is sent
    even if armed were somehow set, and switching off returns to a safe pose."""
    import shutil, tempfile, time
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(Path(__file__).resolve().parents[1] / "config.json", d / "config.json")
    c = Controller(d / "config.json")
    try:
        c.connect("", 0, simulate=True); c.set_home(); c.arm()
        try:
            c.set_view_override(True); raise AssertionError("override allowed while armed")
        except RuntimeError:
            pass
        c.disarm()
        c.set_view_override(True)
        try:
            c.arm(); raise AssertionError("arm allowed while override on")
        except RuntimeError:
            pass
        sent = []
        c.bus.send = lambda frame, _s=c.bus.send: sent.append(frame)   # record anything sent from now on
        c.cfg["control"]["max_rate_deg_s"] = 360
        c.set_mode("pose", {"flex": 45, "add": 0, "rot": 0})          # well past the ~26 deg flexion limit
        time.sleep(1.0)
        s = c.snapshot()
        assert s["view_override"] and abs(s["anat_cmd"]["flex"] - 45) < 0.5, s["anat_cmd"]
        assert s["safety"].get("overridden") and not s["safety"]["ok"]
        with c.lock:
            c.armed = True                 # simulate a bug that arms anyway: the send step must still refuse
        time.sleep(0.3)
        with c.lock:
            c.armed = False
        assert not sent, "a setpoint was sent while the view override was on"
        c.set_view_override(False)
        time.sleep(0.2)
        s = c.snapshot()
        assert s["anat_cmd"]["flex"] < 44 and not s["view_override"]          # back inside the limits
        th, m = c.spm.ik(c.R_cmd, c.theta_cmd)
        assert c.check_pose(c.R_cmd, th, m)[0]
    finally:
        c.shutdown()


def test_gait_preview_flags_refused_phases():
    """The Gait tab plot shades the phases the safety gate refuses.  With the
    v1 design the built-in gait (31.7 deg flexion) is partly refused;
    at half amplitude it is all inside the limits."""
    import shutil, tempfile
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(V1_CFG, d / "config.json")
    c = Controller(d / "config.json")
    try:
        pv = c.gait_preview()
        assert len(pv["ok"]) == len(pv["phase"]) == len(pv["reason"])
        bad = [i for i, ok in enumerate(pv["ok"]) if not ok]
        assert 0 < len(bad) < len(pv["ok"])
        # it is the flexed part of the cycle that is refused (flexion combined with
        # adduction closes the rings sooner than flexion alone: from ~14 deg here)
        assert all(pv["flex"][i] > 10 for i in bad)
        assert all("rings too close" in pv["reason"][i] for i in bad)
        c.set_mode("gait", {"scale": [0.5, 0.5, 0.5]})
        assert all(c.gait_preview()["ok"])
    finally:
        c.shutdown()


def test_remount_keeps_femur_on_head():
    """Geometry tab 'Remount casing': changing only the mount recomputes neutral
    so the femur stays attached to the head (F_E unchanged); option A's mount
    then reproduces option A's neutral (both v1 results).  Without the flag,
    neutral is left as is."""
    import shutil, tempfile
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(V1_CFG, d / "config.json")
    c = Controller(d / "config.json")
    try:
        F_before = c.hip.R_PB @ c.hip.R_neutral
        mountA = {"tilt_down_deg": 38.3, "forward_deg": 10.2, "base_twist_deg": 93.5}
        c.update_config({"mount": mountA, "neutral": dict(c.cfg["neutral"])}, keep_femur=True)
        assert np.allclose(c.hip.R_PB @ c.hip.R_neutral, F_before, atol=1e-5)   # femur still on the head the same way
        n = c.cfg["neutral"]                                                    # ~ option A (-10, 5, 0) up to rounding
        from spm.kinematics import rot_x, rot_y
        R_A = rot_y(math.radians(5)) @ rot_x(math.radians(-10))
        assert np.linalg.norm(rotvec(c.hip.R_neutral @ R_A.T)) < math.radians(0.2), n
        old = dict(c.cfg["neutral"])
        c.update_config({"mount": {"tilt_down_deg": 30.0}}, keep_femur=False)  # plain mount edit: neutral untouched
        assert c.cfg["neutral"] == old
    finally:
        c.shutdown()


def _fake_board_stream(true_hz=230.5, nominal_hz=240, seconds=14.0, gyro_dps=(0.0, 0.0, 10.0), seed=1):
    """Byte stream + PC arrival times that look like the REAL board (HANDOFF 6.1):
    the sensor samples at true_hz, the firmware stamps t_us on a nominal
    1e6/nominal_hz clock and re-anchors it when > 250 ms behind; USB delivers
    in bursty chunks with jitter, and the PC reader stalls 80 ms now and then.
    Returns a list of (bytes, t_rx)."""
    import struct
    from spm.imu_board import build_frame, IMU_FMT, INFO_FMT, T_IMU, T_INFO
    rng = np.random.default_rng(seed)
    gy_lsb = [int(round(g / 70e-3)) for g in gyro_dps]
    nominal_us = 1000000 // nominal_hz
    t_us, chunks, pending = 0, [], bytearray()
    t_next_read = 0.0
    n = int(seconds * true_hz)
    for k in range(n):
        t_prod = k / true_hz                              # true production time, s
        pending += build_frame(T_IMU, struct.pack(IMU_FMT, k, t_us & 0xFFFFFFFF,
                                                  0, 0, 2049, *gy_lsb, 0, 0, 0, 7))
        if k % nominal_hz == 0:
            pending += build_frame(T_INFO, struct.pack(INFO_FMT, 1, nominal_hz, 0.488, 70.0, 1.952,
                                                       0, k, int(t_prod * 1000)))
        t_us += nominal_us                                # bridge.c nominal clock
        if t_prod * 1e6 - t_us > nominal_us * nominal_hz // 4:
            t_us = int(t_prod * 1e6)                      # bridge.c re-anchor (the ~254 ms jumps)
        if t_prod >= t_next_read:                         # host read: everything pending arrives at once
            lat = 0.0005 + rng.exponential(0.002)         # USB + OS latency, always >= 0
            if rng.random() < 0.01:
                lat += 0.08                               # occasional Python / OS stall
            chunks.append((bytes(pending), t_prod + lat))
            pending.clear()
            t_next_read = t_prod + rng.uniform(0.002, 0.012)
    return chunks


def test_sample_clock_recovers_true_rate_and_angle():
    """HANDOFF 6.1 regression: firmware labels 240 Hz, sensor really runs 230.5 Hz.
    The host must measure the true rate from the stream, and integrated gyro
    angles must come out right (they read ~4% short with the nominal clock)."""
    from spm.imu_board import ImuBoard
    b = ImuBoard(None)
    chunks = _fake_board_stream()
    naive = {"t": None, "ang": 0.0}                       # what the OLD code (t_us timing) would integrate

    def naive_cb(t, g, a):
        tb = b.t_board
        if naive["t"] is not None and 0 < tb - naive["t"] < 0.1:
            naive["ang"] += math.degrees(float(np.linalg.norm(g))) * (tb - naive["t"])
        naive["t"] = tb

    integ = None
    t_start = None
    for data, t_rx in chunks:
        b.feed(data, t_rx)
        if integ is None and t_rx > 3.0:                  # after the rate fit has had time to converge
            assert b.clock.converged
            integ = b.gyro_integrator()
            b.sample_listeners.append(naive_cb)
            seq_start = b.seq
    ang = math.degrees(float(np.linalg.norm(integ.stop())))
    true_ang = round(10.0 / 70e-3) * 70e-3 * (b.seq - seq_start) / 230.5   # quantised rate x true duration
    assert abs(b.clock.rate_hz - 230.5) < 0.05, b.clock.rate_hz
    assert abs(ang / true_ang - 1) < 0.002, (ang, true_ang)
    assert naive["ang"] / true_ang < 0.975                # proves the test really contains the bug
    assert b.stats.seq_gaps == 0


def test_sample_clock_survives_seq_gap_and_reboot():
    from spm.imu_board import SampleClock
    c = SampleClock(240)
    for k in range(1000):
        t999 = c.update(k, k / 230.5 + 0.001)
    assert c.converged and abs(c.rate_hz - 230.5) < 0.01
    t = c.update(1100, 1100 / 230.5 + 0.001)              # 100 samples lost on the way
    # dt across the gap = 101 TRUE periods (t itself accumulates, so samples
    # from before convergence were spaced at the nominal period - by design)
    assert abs((t - t999) - 101 / 230.5) < 1e-5
    assert abs((c.update(1101, 1101 / 230.5 + 0.001) - t) - 1 / 230.5) < 1e-6
    c.update(5, 10.0)                                     # seq went backwards = board rebooted
    assert not c.converged and c.t == 0.0


def test_sim_imu_calibration_end_to_end():
    import shutil, tempfile, time
    from spm.controller import Controller
    d = Path(tempfile.mkdtemp())
    shutil.copy(Path(__file__).resolve().parents[1] / "config.json", d / "config.json")
    c = Controller(d / "config.json")
    try:
        c.connect("", 0, simulate=True); c.set_home(); c.arm()
        c.imu.connect("SIM"); time.sleep(1.0)
        c.imu.start_calibration()
        while c.imu.cal_thread.is_alive():
            time.sleep(0.2)
        assert c.imu.board.est.calibrated, c.imu.cal_status
        sim = c.imu.board._ser
        assert np.linalg.norm(rotvec(c.imu.board.est.R_EI @ sim.R_EI.T)) < math.radians(2)
        # The sim samples at 230.5 Hz but labels 240 Hz like the real board:
        # the host must have measured the true rate, so gyro angles match the
        # encoder (scale ~1, not the 0.96 the nominal clock gives).
        assert abs(c.imu.board.clock.rate_hz - 230.5) < 1.0, c.imu.board.clock.rate_hz
        assert abs(np.mean(c.imu.cal_result["gyro_scale"]) - 1) < 0.02, c.imu.cal_result["gyro_scale"]
        c.set_mode("pose", {"flex": 20, "add": 0, "rot": 0}); time.sleep(1.5)
        snap = c.snapshot()["imu"]
        # the simulator's 1.5 deg link compliance must be visible as IMU-vs-encoder tilt error
        assert 0.8 < snap["tilt_err_vs_enc_deg"] < 2.5
        import json
        json.dumps(snap)                                        # must be JSON-serialisable for the GUI
    finally:
        c.estop(); c.shutdown()


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS  {name}")
            except Exception as e:                       # noqa: BLE001
                fails += 1
                print(f"FAIL  {name}: {e!r}")
    print("all passed" if not fails else f"{fails} failed")
    sys.exit(1 if fails else 0)

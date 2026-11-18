"""
torque_estimate.py - Motor torque the SPM needs to move the leg.
(Design-estimate tool, kept in experiments/ on purpose; changes nothing.)

    python experiments/torque_estimate.py                        # 8 kg leg, ring rate 90 deg/s, mount A + today
    python experiments/torque_estimate.py --mass 8 --com 0.386 --ring-rate 90

Model
-----
Leg = one rigid body hanging from the hip centre (= SPM centre), straight
knee.  Default mass split and proportions follow Winter's anthropometric
tables (thigh / shank / foot = 10.0 / 4.65 / 1.45 % body mass, CoM and
radii of gyration as fractions of segment length), scaled to --mass and to
this femur (mechanical length 0.446 m from the CAD, shank ~0.96 x femur):
    CoM ~0.386 m from the hip along the femur's mechanical axis,
    inertia about the hip ~1.9 kg m^2 (any transverse axis), ~0.03 about
    the long axis.
The SPM links and the femur-head part are assumed included in --mass; link
inertia and friction are ignored (covered by the safety factor).

Moment balance about the fixed hip centre:
    tau_spm = I_P alpha + omega x (I_P omega) - r_com x m g
Ring torques by virtual work: the rings move with theta_dot = M omega,
M = Jb^-1 Ja (kinematics.py: Jb theta_dot = Ja omega), so
    tau_spm . omega = tau_ring . theta_dot   ->   tau_ring = M^-T tau_spm.
Motor torque = ring torque / gear ratio (4) / gear efficiency.

Motion: the built-in gait curve, played at the cycle period that makes the
fastest ring reach --ring-rate (deg/s).  Also: the static (gravity-only)
worst case over every pose the safety gate allows on a grid.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import remount_study as S                                        # noqa: E402
from spm.anatomy import HipMapping                               # noqa: E402
from spm.motion import GaitMode                                  # noqa: E402

G = 9.81


def leg_model(mass, femur_len=0.446, shank_ratio=0.96, foot_len=0.26):
    """Winter (2009) segment table -> CoM distance and inertia about the hip."""
    shank_len = shank_ratio * femur_len
    frac = {"thigh": 0.100, "shank": 0.0465, "foot": 0.0145}
    tot = sum(frac.values())
    m = {k: mass * v / tot for k, v in frac.items()}
    # CoM distance from the hip along the leg axis, and radius of gyration about the segment CoM
    d = {"thigh": 0.433 * femur_len,
         "shank": femur_len + 0.433 * shank_len,
         "foot": femur_len + shank_len + 0.05}                 # foot CoM a little below the ankle
    k = {"thigh": 0.323 * femur_len, "shank": 0.302 * shank_len, "foot": 0.475 * foot_len}
    com = sum(m[s] * d[s] for s in m) / mass
    I_tr = sum(m[s] * (d[s] ** 2 + k[s] ** 2) for s in m)      # about a transverse axis through the hip
    I_long = 0.03 * mass / 8.0                                 # about the long axis (small)
    return com, I_tr, I_long


def ring_torques(spm, R, theta, tau_B):
    """tau_ring = M^-T tau_B, M = Jb^-1 Ja (base frame)."""
    W, V = spm.w_world(theta), spm.v_world(R)
    Ja = np.cross(W, V)
    jb = np.einsum("ij,ij->i", np.cross([0, 0, 1.0], W), V)
    M = Ja / jb[:, None]
    return np.linalg.solve(M.T, tau_B)


def run(cfg, label, mass, com, I_tr, I_long, ring_rate, gear, eff, samples=400):
    hip = HipMapping(cfg)
    gate = S.FastGate(cfg)
    spm = gate.spm
    gait = GaitMode(hip)
    I_F = np.diag([I_tr, I_long, I_tr])                         # femur frame: Y = long axis (pelvis axes at neutral)
    g_P = np.array([0, -G, 0])
    ph = np.linspace(0, 100, samples, endpoint=False)
    ang = np.array([gait.angles(p) for p in ph])
    R_P = [hip.anat_matrix(*a) for a in ang]                    # femur orientation in the pelvis frame
    R_B = [hip.effector_from_anat(R) for R in R_P]
    th_home = gate.ik(hip.R_neutral[None])[0][0]
    ok, parts, _ = gate.check(np.array(R_B), th_home)
    # ring angles (continuous)
    th = []
    prev = th_home
    for R in R_B:
        t, _ = spm.ik(R, prev)
        th.append(t)
        prev = t
    th = np.unwrap(np.array(th), axis=0)
    # angular velocity / acceleration in the pelvis frame per unit of phase (cycle = 1)
    dphi = 1.0 / samples

    def omega(i):                                               # rotation rate between neighbours, pelvis frame
        Rm, Rp = R_P[(i - 1) % samples], R_P[(i + 1) % samples]
        from spm.kinematics import rotvec
        return rotvec(Rp @ Rm.T) / (2 * dphi)
    w1 = np.array([omega(i) for i in range(samples)])           # rad per cycle
    a1 = (np.roll(w1, -1, 0) - np.roll(w1, 1, 0)) / (2 * dphi)
    thd1 = (np.roll(th, -1, 0) - np.roll(th, 1, 0)) / (2 * dphi)  # ring rad per cycle
    # pick the cycle period so the fastest ring hits ring_rate
    T = np.degrees(np.abs(thd1).max()) / ring_rate               # s per cycle
    w, a, thd = w1 / T, a1 / T ** 2, thd1 / T
    rows = []
    for i in range(samples):
        Rp = R_P[i]
        r = Rp @ np.array([0, -com, 0])
        I_P = Rp @ I_F @ Rp.T
        tau_grav = -np.cross(r, mass * g_P)
        tau_dyn = I_P @ a[i] + np.cross(w[i], I_P @ w[i])
        tau_P = tau_dyn + tau_grav
        tau_B = hip.R_PB.T @ tau_P
        tr = ring_torques(spm, R_B[i], th[i], tau_B)
        rows.append((np.abs(tr).max(), np.abs(ring_torques(spm, R_B[i], th[i], hip.R_PB.T @ tau_grav)).max(),
                     np.linalg.norm(tau_P), np.linalg.norm(tau_grav)))
    rows = np.array(rows)
    peak = rows[:, 0].max()
    i = int(rows[:, 0].argmax())
    print(f"\n== {label}")
    print(f"   gait cycle period for a {ring_rate:.0f} deg/s peak ring rate: {T:.2f} s "
          f"(motor {ring_rate * gear / 6:.0f} rpm); poses the gate refuses in this cycle: {int((~ok).sum())}/{samples}")
    print(f"   hip torque the SPM must supply: peak {rows[:, 2].max():.1f} N m (gravity part {rows[:, 3].max():.1f} N m)")
    print(f"   ring torque: peak {peak:.1f} N m at {ph[i]:.0f}% of the cycle (gravity alone {rows[:, 1].max():.1f} N m)")
    print(f"   MOTOR torque = ring / {gear:g} / eff {eff:g}: peak {peak / gear / eff:.2f} N m"
          f"   (x1.5 safety: {1.5 * peak / gear / eff:.2f}, x2: {2 * peak / gear / eff:.2f} N m)")
    return peak / gear / eff


def load_curve(path=HERE / "motor_curve_57_similar.csv"):
    """(rpm, N m) arrays of a pull-out torque curve (lines starting '#' are notes)."""
    rows = [l.split(",") for l in Path(path).read_text().splitlines()
            if l.strip() and not l.startswith("#") and not l.startswith("rpm")]
    rpm = np.array([float(r[0]) for r in rows])
    tq = np.array([float(r[1]) for r in rows]) / 100.0           # N cm -> N m
    return rpm, tq


def curve_margin(cfg, mass, com, I_tr, I_long, ring_rate, gear, eff, curve, samples=400):
    """Worst ratio (available torque at that motor's speed) / (torque needed),
    over every motor and every instant of the gait timed for `ring_rate`.
    Returns (margin, details of the worst instant)."""
    hip = HipMapping(cfg)
    gate = S.FastGate(cfg)
    spm = gate.spm
    gait = GaitMode(hip)
    from spm.kinematics import rotvec
    I_F = np.diag([I_tr, I_long, I_tr])
    g_P = np.array([0, -G, 0])
    ph = np.linspace(0, 100, samples, endpoint=False)
    R_P = [hip.anat_matrix(*gait.angles(p)) for p in ph]
    R_B = [hip.effector_from_anat(R) for R in R_P]
    th, prev = [], gate.ik(hip.R_neutral[None])[0][0]
    for R in R_B:
        t, _ = spm.ik(R, prev)
        th.append(t)
        prev = t
    th = np.unwrap(np.array(th), axis=0)
    d = 1.0 / samples
    w1 = np.array([rotvec(R_P[(i + 1) % samples] @ R_P[i - 1].T) / (2 * d) for i in range(samples)])
    a1 = (np.roll(w1, -1, 0) - np.roll(w1, 1, 0)) / (2 * d)
    thd1 = (np.roll(th, -1, 0) - np.roll(th, 1, 0)) / (2 * d)
    T = np.degrees(np.abs(thd1).max()) / ring_rate
    w, a, thd = w1 / T, a1 / T ** 2, thd1 / T
    rpm_c, tq_c = curve
    worst = (np.inf, None)
    for i in range(samples):
        Rp = R_P[i]
        I_P = Rp @ I_F @ Rp.T
        tauP = I_P @ a[i] + np.cross(w[i], I_P @ w[i]) - np.cross(Rp @ np.array([0, -com, 0]), mass * g_P)
        tr = ring_torques(spm, R_B[i], th[i], hip.R_PB.T @ tauP)
        for k in range(3):
            need = abs(tr[k]) / gear / eff                      # motor torque, N m
            rpm = abs(np.degrees(thd[i, k])) * gear / 6.0      # motor speed, rpm
            have = float(np.interp(rpm, rpm_c, tq_c))          # held flat below the first point / beyond the last
            ratio = have / max(need, 1e-9)
            if ratio < worst[0]:
                worst = (ratio, {"phase": ph[i], "motor": k + 1, "rpm": rpm, "need": need, "have": have})
    return worst[0], worst[1], T


def static_worst(cfg, label, mass, com, gear, eff):
    """Gravity-only ring torque over every allowed pose on a hip-angle grid."""
    hip = HipMapping(cfg)
    gate = S.FastGate(cfg)
    spm = gate.spm
    g_P = np.array([0, -G, 0])
    th_home = gate.ik(hip.R_neutral[None])[0][0]
    worst = (0.0, None)
    for f in np.arange(-40, 121, 5.0):
        for ad in np.arange(-45, 46, 5.0):
            for r in np.arange(-60, 61, 10.0):
                Rp = hip.anat_matrix(f, ad, r)
                R = hip.effector_from_anat(Rp)
                if not gate.check(R[None], th_home)[0][0]:
                    continue
                th, _ = spm.ik(R, th_home)
                tau = hip.R_PB.T @ (-np.cross(Rp @ np.array([0, -com, 0]), mass * g_P))
                t = np.abs(ring_torques(spm, R, th, tau)).max()
                if t > worst[0]:
                    worst = (t, (f, ad, r))
    print(f"   static (holding) worst case over all allowed poses: ring {worst[0]:.1f} N m -> motor "
          f"{worst[0] / gear / eff:.2f} N m at flex/add/rot {worst[1]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mass", type=float, default=8.0, help="leg assembly mass, kg")
    ap.add_argument("--com", type=float, default=None, help="CoM distance from the hip centre, m (default: Winter model)")
    ap.add_argument("--ring-rate", type=float, default=90.0, help="peak ring rate, deg/s")
    ap.add_argument("--eff", type=float, default=0.9, help="gear efficiency (18:72 spur/helical stage)")
    ap.add_argument("--gear", type=float, default=None, help="gear ratio (default: config.json control.gear_ratio)")
    args = ap.parse_args()
    com, I_tr, I_long = leg_model(args.mass)
    if args.com:
        I_tr *= (args.com / com) ** 2
        com = args.com
    print(f"leg {args.mass:g} kg: CoM {com:.3f} m from the hip, I_hip {I_tr:.2f} kg m^2 (transverse), {I_long:.3f} (long axis)")
    print(f"for scale: gravity torque with the leg horizontal = {args.mass * G * com:.1f} N m, "
          f"at 30 deg flexion = {args.mass * G * com * 0.5:.1f} N m")
    base = json.loads((ROOT / "config.json").read_text())
    gear = args.gear or base["control"]["gear_ratio"]
    for label, neutral, mount in (
            ("mount A (recommended remount)", {"tilt_x_deg": -10, "tilt_y_deg": 5, "twist_deg": 0},
             {"tilt_down_deg": 38.3, "forward_deg": 10.2, "base_twist_deg": 93.5}),
            ("today's mount (gait only reachable to x0.55; numbers for the full curve are hypothetical)", None, None)):
        cfg = json.loads(json.dumps(base))
        if neutral:
            cfg["neutral"].update(neutral)
            cfg["mount"].update(mount)
        run(cfg, label, args.mass, com, I_tr, I_long, args.ring_rate, gear, args.eff)
        # torque needed vs the speed-dependent curve, instant by instant (motor_curve_57_similar.csv)
        m, det, _ = curve_margin(cfg, args.mass, com, I_tr, I_long, args.ring_rate, gear, args.eff, load_curve())
        print(f"   vs the similar-motor pull-out curve: worst margin x{m:.2f} (motor {det['motor']} at "
              f"{det['phase']:.0f}% of the cycle, {det['rpm']:.0f} rpm: needs {det['need']:.2f}, curve {det['have']:.2f} N m)")
        static_worst(cfg, label, args.mass, com, gear, args.eff)


if __name__ == "__main__":
    main()

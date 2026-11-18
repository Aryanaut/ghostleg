"""
rom_check.py - Can a given config (geometry + mount + neutral + CAD collision
limits) do the required range of motion?  Uses the controller's REAL safety
gate (Controller.check_pose), so it answers exactly what the rig would allow.
(Analysis tool, kept in experiments/ on purpose.)

    python experiments/rom_check.py config_v2.json
    python experiments/rom_check.py config.json --mass 4

Reports
  * single-axis range from neutral (flexion / adduction / int. rotation, both
    signs) and the check that stops each direction;
  * the built-in gait: share of the cycle refused, and the largest scale of
    the whole curve that passes (x1.00 = the full textbook gait);
  * a "required ROM" box (default: normal gait ranges with margin, flex -15..35,
    add -10..12, rot -10..10) - fraction of poses allowed;
  * motor torque margin over the gait vs motor_curve_57_similar.csv
    (torque_estimate.py model) at the config's gear ratio.
"""
from __future__ import annotations

import argparse
import collections
import io
import contextlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
from spm.controller import Controller                           # noqa: E402
from spm.motion import GaitMode                                  # noqa: E402


def gate_fn(c):
    def gate(f, a, r, th_ref):
        R = c.hip.effector_from_anat(c.hip.anat_matrix(f, a, r))
        th, m = c.spm.ik(R, th_ref)
        ok, why, met = c.check_pose(R, th, m)
        return ok, why, th
    return gate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--mass", type=float, nargs="*", default=[4.0, 8.0], help="leg masses for the torque check, kg")
    ap.add_argument("--ring-rate", type=float, default=90.0)
    args = ap.parse_args()
    d = Path(tempfile.mkdtemp())
    shutil.copy(args.config, d / "config.json")                  # never let the Controller touch the original
    c = Controller(d / "config.json")
    c.shutdown()
    cfg = c.cfg
    gate = gate_fn(c)
    print(f"config {args.config}: alpha1 {cfg['geometry']['alpha1_deg']}, gear {cfg['control']['gear_ratio']}:1, "
          f"mount {cfg['mount']['tilt_down_deg']}/{cfg['mount']['forward_deg']}/{cfg['mount']['base_twist_deg']}, "
          f"joint limits {'from ' + cfg['limits']['joint_limits'].get('_source', '?')[:60] if 'joint_limits' in cfg['limits'] else 'NONE'}")
    th0 = c.theta_home
    ok0, why0, _ = gate(0, 0, 0, th0)
    print(f"neutral passes the gate: {ok0} {why0}")

    print("\nsingle-axis range from neutral (first refused pose, and why):")
    for name, ax in (("flexion", 0), ("adduction", 1), ("int. rotation", 2)):
        res = []
        for sgn in (1, -1):
            th = th0.copy()
            hit = (f"{sgn * 120:+.0f}+", "free")
            for x in np.arange(0.5, 120.01, 0.5) * sgn:
                a = [0.0, 0.0, 0.0]
                a[ax] = x
                ok, why, th_new = gate(*a, th)
                if not ok:
                    hit = (f"{x - 0.5 * sgn:+.1f}", why.split(" (")[0].split(",")[0])
                    break
                th = th_new
            res.append(f"{hit[0]:>7} ({hit[1]})")
        print(f"  {name:14s} +: {res[0]:45s} -: {res[1]}")

    g = GaitMode(c.hip)
    phases = np.arange(0, 100, 0.5)
    ang = np.array([g.angles(p) for p in phases])

    def gait_ok(scale):
        th, bad = th0.copy(), collections.Counter()
        for a in ang * scale:
            ok, why, th_new = gate(*a, th)
            if not ok:
                bad[why.split(" (")[0].split(" at")[0]] += 1
            elif np.all(np.isfinite(th_new)):
                th = th_new
        return bad
    bad = gait_ok(1.0)
    print(f"\nbuilt-in gait (flex {ang[:, 0].min():.0f}..{ang[:, 0].max():.0f}, add {ang[:, 1].min():.0f}..{ang[:, 1].max():.0f}, "
          f"rot {ang[:, 2].min():.0f}..{ang[:, 2].max():.0f}): {100 * sum(bad.values()) / len(phases):.0f}% of the cycle refused"
          + (f" ({dict(bad)})" if bad else ""))
    best = 0.0
    for s in np.arange(0.05, 3.001, 0.05):
        if gait_ok(s):
            break
        best = s
    print(f"largest scale of the whole gait that passes: x{best:.2f}")

    n = bad_n = 0
    reasons = collections.Counter()
    for f in np.arange(-15, 35.1, 2.5):
        for a in np.arange(-10, 12.1, 2.0):
            for r in np.arange(-10, 10.1, 2.5):
                ok, why, _ = gate(f, a, r, th0)
                n += 1
                if not ok:
                    bad_n += 1
                    reasons[why.split(" (")[0].split(" at")[0]] += 1
    print(f"required-ROM box flex -15..35, add -10..12, rot -10..10: {n - bad_n}/{n} poses allowed "
          f"({100 * (n - bad_n) / n:.0f}%)" + (f"; refused: {dict(reasons)}" if reasons else ""))

    import torque_estimate as T
    curve = T.load_curve()
    gear = cfg["control"]["gear_ratio"]
    print(f"\nmotor torque vs the similar-motor curve, gait at {args.ring_rate:g} deg/s peak ring rate, {gear:g}:1:")
    for mass in args.mass:
        com, I_tr, I_long = T.leg_model(mass)
        with contextlib.redirect_stdout(io.StringIO()):
            m, det, Tc = T.curve_margin(cfg, mass, com, I_tr, I_long, args.ring_rate, gear, 0.9, curve, samples=200)
        print(f"  {mass:g} kg: worst margin x{m:.2f} (motor {det['motor']}, {det['rpm']:.0f} rpm, needs {det['need']:.2f} N m;"
              f" gait cycle {Tc:.1f} s)")


if __name__ == "__main__":
    main()

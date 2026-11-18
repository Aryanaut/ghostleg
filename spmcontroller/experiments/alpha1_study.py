"""
alpha1_study.py - How the proximal-link angle alpha1 (ring axis -> middle-joint
axis) changes reach, usable range of motion, gait headroom and motor torque.
(Design-study tool, kept in experiments/ on purpose; changes nothing.)

    python experiments/alpha1_study.py

Everything except alpha1 is kept as designed (alpha2 = 90, beta = 90, the head
joints' eta, branch -1) with mount option A (neutral tilt_x -10, tilt_y +5).
CAVEAT: the elbow/wrist fold limits were measured on the alpha1 = 40 CAD; a
different alpha1 means different arm parts, so they are switched OFF here, and
the arm-contact ring separation (45 deg incl. margin) is assumed unchanged.

Columns
  reach     largest flexion / adduction / int. rotation (each sign) with ANY
            IK solution (the mechanism can physically get there)
  usable    same, but also passing the controller's checks: leg margin,
            parallel / serial singularity indices, ring separation >= 45 deg
            (arm contact), ring excursion
  gait      largest scale of the built-in gait passing those checks
  torque    peak motor torque for the gait at 90 deg/s peak ring rate,
            4 kg leg, 6:1 gears, 90 % efficiency (torque_estimate.py model)
  neutral   parallel / serial index at neutral (1 = far from singular)
"""
from __future__ import annotations

import contextlib
import io
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import remount_study as S                                    # noqa: E402
import torque_estimate as T                                  # noqa: E402
from spm.anatomy import HipMapping                           # noqa: E402
from spm.motion import GaitMode                              # noqa: E402


def make_cfg(base, a1):
    cfg = json.loads(json.dumps(base))
    cfg["geometry"]["alpha1_deg"] = [a1] * 3
    cfg["neutral"].update({"tilt_x_deg": -10, "tilt_y_deg": 5, "twist_deg": 0})
    cfg["mount"].update({"tilt_down_deg": 38.3, "forward_deg": 10.2, "base_twist_deg": 93.5})
    jl = cfg["limits"]["joint_limits"]
    jl["elbow_deg"] = [[-180, 180]] * 3                       # CAD fold limits only valid for alpha1 = 40
    jl["wrist_deg"] = [[-180, 180]] * 3
    return cfg


def one_axis(cfg, gate, check):
    hip = HipMapping(cfg)
    th_home = gate.ik(hip.R_neutral[None])[0][0]
    out = []
    for axis in range(3):
        for sgn in (1, -1):
            xs = np.arange(0, 150.01, 0.5) * sgn
            A = np.zeros((len(xs), 3))
            A[:, axis] = xs
            R = np.array([hip.effector_from_anat(hip.anat_matrix(*a)) for a in A])
            if check:
                ok = gate.check(R, th_home)[0]
            else:
                th, margin, _ = gate.ik(R)
                ok = margin.min(1) >= 0
            bad = np.nonzero(~ok)[0]
            out.append(xs[bad[0] - 1] if len(bad) else xs[-1])
            if len(bad) and bad[0] == 0:
                out[-1] = float("nan")
    return out                                              # flex+, flex-, add+, add-, rot+, rot-


def gait_scale(cfg, gate):
    hip = HipMapping(cfg)
    g = GaitMode(hip)
    ang = np.array([g.angles(p) for p in np.linspace(0, 100, 100, endpoint=False)])
    th_home = gate.ik(hip.R_neutral[None])[0][0]
    best = 0.0
    for s in np.arange(0.05, 3.001, 0.05):
        R = np.array([hip.effector_from_anat(hip.anat_matrix(*(s * a))) for a in ang])
        if not gate.check(R, th_home)[0].all():
            break
        best = s
    return best


def main():
    base = json.loads((ROOT / "config.json").read_text())
    com, I_tr, I_long = T.leg_model(4.0)
    print(__doc__.split("Columns")[0].strip().splitlines()[-1])
    print(f"{'a1':>4} | {'reach: flex+ flex- add+ add- rot+ rot-':^38} | {'usable: flex+ flex- add+ add- rot+ rot-':^39} |"
          f" gait  | motor N m | neutral par/ser | home ring sep")
    for a1 in (30, 35, 40, 45, 50, 55, 60):
        cfg = make_cfg(base, a1)
        gate = S.FastGate(cfg)
        reach = one_axis(cfg, gate, check=False)
        use = one_axis(cfg, gate, check=True)
        gs = gait_scale(cfg, gate)
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                tq = T.run(cfg, "", 4.0, com, I_tr, I_long, 90.0, 6.0, 0.9)
            except Exception:
                tq = float("nan")
        hip = HipMapping(cfg)
        th, m = gate.spm.ik(hip.R_neutral)
        met = gate.spm.singularity_metrics(hip.R_neutral, th)
        sep = min(abs((th[i] - th[j] + np.pi) % (2 * np.pi) - np.pi) for i, j in ((0, 1), (1, 2), (0, 2)))
        f = lambda v: f"{v:+5.0f}" if np.isfinite(v) else "  n/a"
        print(f"{a1:4d} | {' '.join(f(v) for v in reach):38s} | {' '.join(f(v) for v in use):39s} |"
              f" x{gs:4.2f} | {tq if gs >= 1 else float('nan'):9.2f} | {met['parallel_index']:.2f} / {met['serial_index']:.2f}"
              f"     | {np.degrees(sep):5.1f}")


if __name__ == "__main__":
    main()

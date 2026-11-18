"""
remount_verify.py - Check a few remount candidates from remount_study.py more
thoroughly.  (Design-study tool, experiments/ only; changes nothing.)

For each candidate neutral R_n = Ry(tilt_y) Rx(tilt_x) Rz(twist):
  * robustness: worst gait scale over mounts built 5 deg off in tilt_x /
    tilt_y and 10 deg off in twist (build tolerance);
  * single-axis ROM from neutral (flex / add / int. rot, each sign);
  * CAD part clearances (experiments/cad_collision.py) of the pose-dependent
    pairs along the full gait at x1.0 and at the candidate's max scale;
  * the mount angles for config.json, and the equivalent change of the
    femur-head part on the femur (if the casing mount stays as it is).
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import remount_study as S                                          # noqa: E402
from spm.anatomy import HipMapping                                 # noqa: E402
from spm.kinematics import rotvec                                  # noqa: E402
from spm.motion import GaitMode                                    # noqa: E402

CANDIDATES = {                       # name: (tilt_x, tilt_y, twist)  - from remount_study.py
    "today": (15.449, -9.108, 0),
    "symmetric home": (0, 0, 0),
    "A: robust x1.45 (recommended)": (-10, 5, 0),
    "B: robust x1.3, smaller change": (-5, 5, 0),
    "peak x1.9 (fragile)": (-15, 5, 20),
}


def main():
    cfg = json.loads((ROOT / "config.json").read_text())
    hip = HipMapping(cfg)
    gate = S.FastGate(cfg)
    F_E = (hip.R_PB @ hip.R_neutral).T
    gait = GaitMode(hip)
    ang = np.array([gait.angles(p) for p in np.linspace(0, 100, 100, endpoint=False)])
    A = lambda s: np.array([F_E @ hip.anat_matrix(*(s * a)) @ F_E.T for a in ang])
    scales = np.round(np.arange(0.05, 2.0001, 0.05), 3)
    A_s = {s: A(s) for s in scales}

    def score(R_n):
        th_home = gate.ik(R_n[None])[0][0]
        if not gate.check(R_n[None], th_home)[0][0]:
            return 0.0
        best = 0.0
        for s in scales:
            if not gate.check(np.einsum("ij,njk->nik", R_n, A_s[s]), th_home)[0].all():
                return best
            best = float(s)
        return best

    def rom(R_n):
        th_home = gate.ik(R_n[None])[0][0]
        out = {}
        for k, axis in (("flex", 0), ("add", 1), ("rot", 2)):
            for sgn in (-1, 1):
                xs = np.arange(0, 120.01, 0.5) * sgn
                ab = np.zeros((len(xs), 3)); ab[:, axis] = xs
                R = np.array([R_n @ F_E @ hip.anat_matrix(*a) @ F_E.T for a in ab])
                ok, parts, _ = gate.check(R, th_home)
                bad = np.nonzero(~ok)[0]
                if len(bad):
                    why = [n for n, v in parts.items() if not v[bad[0]]]
                    out[f"{k}{'+' if sgn > 0 else '-'}"] = f"{xs[bad[0]] - 0.5 * sgn:+.1f} ({why[0]})"
                else:
                    out[f"{k}{'+' if sgn > 0 else '-'}"] = ">120"
        return out

    import cad_collision as K
    rig = K.Rig(ROOT / "bone-in SPM.step")
    gp = K.general_pairs(rig)

    def clearance_along_gait(R_n, s):
        worst = (99.0, None)
        for M in A_s[min(scales, key=lambda x: abs(x - s))]:
            R = R_n @ M
            cl = rig.clearances(R, gp)
            if cl is None:
                return (-1.0, "unreachable")
            k = min(cl, key=cl.get)
            if cl[k] < worst[0]:
                worst = (cl[k], f"{rig.label(k[0])} <-> {rig.label(k[1])}")
        return worst

    R_PB_now = hip.R_PB
    report = {}
    for name, (tx, ty, tw) in CANDIDATES.items():
        R_n = S.R_n_of(tx, ty, tw)
        s = score(R_n)
        neigh = [score(S.R_n_of(tx + dx, ty + dy, tw + dt))
                 for dx, dy, dt in itertools.product((-5, 0, 5), (-5, 0, 5), (-10, 0, 10))]
        R_PB = F_E.T @ R_n.T
        md = S.mount_angles(R_PB, hip.side)
        # equivalent: keep the casing mount, re-angle the femur-head part on the femur.
        # head axes in the femur frame: H = R_PB @ R_n (old) -> R_PB_now @ R_n (new, casing unchanged)
        dH = (R_PB_now @ R_n) @ (R_PB_now @ hip.R_neutral).T
        rv = rotvec(dH)
        ang_dH = math.degrees(np.linalg.norm(rv))
        ax = rv / max(np.linalg.norm(rv), 1e-12)
        c1 = clearance_along_gait(R_n, 1.0) if s >= 1.0 else (None, "gait not reachable")
        cmax = clearance_along_gait(R_n, s) if s >= 1.0 else (None, "")
        report[name] = {"neutral": [tx, ty, tw], "gait_scale": s, "worst_scale_within_build_tolerance": min(neigh),
                        "rom": rom(R_n), "mount": [round(x, 1) for x in md],
                        "head_part_change_deg": round(ang_dH, 1),
                        "head_part_change_axis_pelvis_XYZ": [round(x, 3) for x in ax],
                        "cad_clearance_gait_x1": c1, "cad_clearance_gait_xmax": cmax}
        r = report[name]
        print(f"\n== {name}: neutral tilt_x {tx} tilt_y {ty} twist {tw}")
        print(f"   gait fits up to x{s:.2f}; worst within +-5 deg / +-10 deg build error: x{min(neigh):.2f}")
        print(f"   ROM from neutral: " + ", ".join(f"{k} {v}" for k, v in r["rom"].items()))
        print(f"   mount (config 'mount'): tilt_down {md[0]:.1f}, forward {md[1]:.1f}, base_twist {md[2]:.1f}")
        print(f"   OR keep the casing and re-angle the head on the femur by {ang_dH:.1f} deg about pelvis-axis "
              f"{np.round(ax, 2)} (X ant, Y sup, Z lat)")
        print(f"   CAD clearance of pose-dependent pairs along gait x1: {c1};  at x{s:.2f}: {cmax}")
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "remount_candidates.json").write_text(json.dumps(report, indent=1, default=str))


if __name__ == "__main__":
    main()

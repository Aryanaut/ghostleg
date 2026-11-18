"""arm_angle_check.py - smallest ring separation (= largest arm-arm contact angle) each mount option needs for the
full gait cycle, nominal, with x1.2 / x1.3 gait, and worst case over +-5 deg / +-10 deg build error.
Run from the project folder: python experiments/arm_angle_check.py   (design-study tool, experiments/ only)"""
# Max arm-arm contact angle (ring separation at contact) that still lets the full gait cycle run.
import sys, json, math, itertools
sys.path.insert(0, "experiments"); sys.path.insert(0, ".")
import numpy as np
import remount_study as S
from spm.anatomy import HipMapping
from spm.motion import GaitMode
cfg = json.loads(open("config.json").read())
cfg["limits"]["min_ring_separation_deg"] = 0.0          # ring separation is what we solve for
hip = HipMapping(cfg); gate = S.FastGate(cfg)
F_E = (hip.R_PB @ hip.R_neutral).T
gait = GaitMode(hip)
ang = np.array([gait.angles(p) for p in np.linspace(0, 100, 400, endpoint=False)])

def need(R_n, s=1.0, flex_add=0.0):
    """min ring separation along the (scaled) gait, and whether everything else passes."""
    a = ang * s; a[:, 0] = a[:, 0] + np.sign(a[:, 0]) * 0 + flex_add * (a[:, 0] > 0)
    R = np.einsum("ij,njk->nik", R_n, np.array([F_E @ hip.anat_matrix(*x) @ F_E.T for x in a]))
    th_home = gate.ik(R_n[None])[0][0]
    ok, parts, _ = gate.check(R, th_home)
    th = gate.ik(R)[0]
    seps = [np.degrees(np.abs((th[:, i] - th[:, j] + np.pi) % (2 * np.pi) - np.pi)) for i, j in ((0, 1), (1, 2), (0, 2))]
    smin = np.min(seps, axis=0)
    k = int(np.argmin(smin))
    return float(smin.min()), bool(ok.all()), {n: int((~v).sum()) for n, v in parts.items() if (~v).any()}, ang[k] * s

cands = {"today (as built)": (15.449, -9.108, 0), "symmetric home": (0, 0, 0),
         "A (recommended)": (-10, 5, 0), "B (smaller change)": (-5, 5, 0)}
print("Arms touch today at 43.5 deg ring separation; controller adds 1.5 deg -> limit 45.\n")
print("mount                 gait x1.0     x1.2     x1.3   | worst over +-5deg build error (x1.0)  | other checks at x1.0")
for name, (tx, ty, tw) in cands.items():
    R_n = S.R_n_of(tx, ty, tw)
    r = [need(R_n, s) for s in (1.0, 1.2, 1.3)]
    worst = min(need(S.R_n_of(tx + dx, ty + dy, tw + dt))[0]
                for dx, dy, dt in itertools.product((-5, 0, 5), (-5, 0, 5), (-10, 0, 10)))
    print(f"{name:20s}  {r[0][0]:6.1f}   {r[1][0]:6.1f}   {r[2][0]:6.1f}   |   {worst:6.1f}"
          f"                              | {'all pass' if r[0][1] else r[0][2]}  (tightest at flex/add/rot {np.round(r[0][3], 1)})")

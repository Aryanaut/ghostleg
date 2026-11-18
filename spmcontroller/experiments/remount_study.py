"""
remount_study.py - Which way should the SPM be mounted so the leg can do gait?
(Design-study tool, kept in experiments/ on purpose.  Nothing here changes
config.json; it prints / plots candidates for the user to choose from.)

    python experiments/remount_study.py              # scan + report + plot
    python experiments/remount_study.py --step 2.5   # finer scan

The idea
--------
The femur is bolted to the femur-head part, so the femur's orientation in the
EFFECTOR frame is fixed by the hardware:  F_E = (R_PB @ R_neutral)^T  for the
current config (pelvis axes at neutral = femur axes).  For a hip rotation
R_anat (pelvis frame) the effector must then be at

    R_BE = R_n @ A,     A = F_E @ R_anat @ F_E^T      (hip motion seen from the head)

where R_n = R_BE at neutral.  Remounting the base on the pelvis changes ONLY
R_n (R_PB = F_E^T @ R_n^T); the motion A the gait asks for is the same.
Everything the safety gate checks (reach, singularities, ring separation =
arm contact, elbow / wrist folds measured in the CAD) is a function of R_BE
alone, so a mount is good if  R_n @ A(t)  stays inside the allowed set for
the whole gait cycle, with room to spare.

Spinning the whole SPM about its own ring axis (R_n -> Rz(a) @ R_n) moves all
three rings by the same angle and changes none of those checks, so R_n has
only three useful parameters:
    R_n = Ry(tilt_y) @ Rx(tilt_x) @ Rz(twist)
(tilt of the femur-head axis away from the ring axis at neutral, and twist of
the head about its own axis).  Today: tilt_x 15.45, tilt_y -9.11, twist 0.

Score of a mount = the largest factor s for which the gait curve scaled by s
(all three angles) passes the gate at every sample.  s >= 1 means the whole
built-in gait fits; s = 1.3 means 30% headroom.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spm.anatomy import HipMapping                      # noqa: E402
from spm.kinematics import CoaxialSPM, SPMGeometry, rot_x, rot_y, rot_z, rotvec   # noqa: E402
from spm.motion import GaitMode                         # noqa: E402

OUT = Path(__file__).resolve().parent / "results"


class FastGate:
    """controller.check_pose, vectorised over many poses (same thresholds).
    max_tilt_deg is NOT applied (it is only a backstop now; reported instead)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.spm = CoaxialSPM(SPMGeometry.from_dict(cfg["geometry"]))
        g = self.spm.geom
        self.a1, self.a2 = g.a1, g.a2
        self.branch = np.asarray(g.branch, float)
        self.vloc = self.spm.v_local()                   # (3 legs, 3)
        self.lim = cfg["limits"]
        jl = self.lim["joint_limits"]
        self.R_ref = np.array(jl["ref_R_BE"], float) if "ref_R_BE" in jl else HipMapping(cfg).R_neutral
        self.th_ref, _ = self.spm.ik(self.R_ref)
        self.W_ref = self.spm.w_world(self.th_ref)
        self.V_ref = self.spm.v_world(self.R_ref)
        self.el = np.array(jl["elbow_deg"], float)
        self.wr = np.array(jl["wrist_deg"], float)
        self.jm = float(jl.get("margin_deg", 0))

    def ik(self, R):                                      # R: (N,3,3) -> theta (N,3), margin (N,3), V (N,3,3)
        V = np.einsum("nij,lj->nli", R, self.vloc)
        A = np.sin(self.a1) * V[..., 0]
        B = np.sin(self.a1) * V[..., 1]
        C = np.cos(self.a2) + np.cos(self.a1) * V[..., 2]
        rho = np.hypot(A, B)
        ratio = C / np.maximum(rho, 1e-12)
        margin = 1 - np.abs(ratio)
        th = np.arctan2(B, A) + self.branch * np.arccos(np.clip(ratio, -1, 1))
        return th, margin, V

    def W(self, th):
        return np.stack([np.sin(self.a1) * np.cos(th), np.sin(self.a1) * np.sin(th),
                         -np.cos(self.a1) * np.ones_like(th)], axis=-1)

    def check(self, R, th_home):
        """Boolean (N,) pass/fail plus a dict of per-check pass arrays."""
        lim = self.lim
        th, margin, V = self.ik(R)
        W = self.W(th)
        ok = {}
        ok["reach"] = margin.min(1) >= lim["min_leg_margin"]
        Ja = np.cross(W, V)
        rows = Ja / np.maximum(np.linalg.norm(Ja, axis=-1, keepdims=True), 1e-12)
        ok["parallel"] = np.abs(np.linalg.det(rows)) >= lim["min_parallel_index"]
        zw = np.cross(np.array([0, 0, 1.0]), W)
        jb = np.einsum("nij,nij->ni", zw, V)
        ok["serial"] = (np.abs(jb) / np.maximum(np.linalg.norm(zw, axis=-1), 1e-12)).min(1) >= lim["min_serial_index"]
        seps = []
        for i, j in ((0, 1), (1, 2), (0, 2)):
            d = np.abs((th[:, i] - th[:, j] + np.pi) % (2 * np.pi) - np.pi)
            seps.append(np.degrees(d))
        ok["ring_sep"] = np.min(seps, axis=0) >= lim["min_ring_separation_deg"]
        exc = np.degrees(np.abs((th - th_home + np.pi) % (2 * np.pi) - np.pi)).max(1)
        ok["excursion"] = exc <= lim["max_ring_excursion_deg"]
        # elbow / wrist fold angles relative to the CAD reference pose
        dz = th - self.th_ref
        c, s = np.cos(dz), np.sin(dz)
        v_ref = np.stack([c * self.V_ref[:, 0] - s * self.V_ref[:, 1],
                          s * self.V_ref[:, 0] + c * self.V_ref[:, 1],
                          np.broadcast_to(self.V_ref[:, 2], c.shape)], axis=-1)
        elbow = np.degrees(np.arctan2(np.einsum("nij,nij->ni", np.cross(v_ref, V), W), np.einsum("nij,nij->ni", v_ref, V)))
        w_ref = np.einsum("nij,jk,lk->nli", R, self.R_ref.T, self.W_ref)
        wrist = np.degrees(np.arctan2(np.einsum("nij,nij->ni", np.cross(w_ref, W), V), np.einsum("nij,nij->ni", w_ref, W)))
        jok = np.ones(len(R), bool)
        for ang, L in ((elbow, self.el), (wrist, self.wr)):
            for i in range(3):
                lo, hi = L[i]
                if lo > -180:
                    jok &= ang[:, i] - lo >= self.jm
                if hi < 180:
                    jok &= hi - ang[:, i] >= self.jm
        ok["joints"] = jok
        allok = np.ones(len(R), bool)
        for v in ok.values():
            allok &= v
        tilt = np.degrees(np.arccos(np.clip(R[:, 2, 2], -1, 1)))
        return allok, ok, tilt


def R_n_of(tx, ty, tw):
    return rot_y(math.radians(ty)) @ rot_x(math.radians(tx)) @ rot_z(math.radians(tw))


def mount_angles(R_PB, side=1):
    """R_PB -> (tilt_down, forward, base_twist) in anatomy.py's convention."""
    z_b = R_PB[:, 2]
    tilt = math.degrees(math.asin(-z_b[1]))
    fwd = math.degrees(math.atan2(z_b[0], z_b[2] * side))
    x_def = np.array([1.0, 0, 0]) - z_b[0] * z_b
    x_def /= np.linalg.norm(x_def)
    tw = math.degrees(math.atan2(np.cross(x_def, R_PB[:, 0]) @ z_b, x_def @ R_PB[:, 0]))
    return tilt, fwd, tw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=5.0, help="scan step for tilt_x / tilt_y (deg)")
    ap.add_argument("--max-tilt", type=float, default=50.0, help="largest neutral tilt to consider (deg)")
    ap.add_argument("--samples", type=int, default=100, help="gait samples per cycle")
    args = ap.parse_args()

    cfg = json.loads((ROOT / "config.json").read_text())
    hip = HipMapping(cfg)
    gate = FastGate(cfg)
    F_E = (hip.R_PB @ hip.R_neutral).T                     # femur (= pelvis at neutral) axes in the effector frame
    gait = GaitMode(hip)
    ang = np.array([gait.angles(p) for p in np.linspace(0, 100, args.samples, endpoint=False)])

    def A_of(scale):
        return np.array([F_E @ hip.anat_matrix(*(scale * a)) @ F_E.T for a in ang])

    scales = np.round(np.arange(0.05, 2.0001, 0.05), 3)
    A_s = {s: A_of(s) for s in scales}

    def score(R_n):
        th_home, _, _ = gate.ik(R_n[None])
        ok0, _, _ = gate.check(R_n[None], th_home[0])
        if not ok0[0]:
            return 0.0, None
        best = 0.0
        for s in scales:                                   # largest s with every smaller s also passing
            ok, parts, tilt = gate.check(np.einsum("ij,njk->nik", R_n, A_s[s]), th_home[0])
            if not ok.all():
                fail = {k: int((~v).sum()) for k, v in parts.items() if (~v).any()}
                return best, fail
            best = float(s)
        return best, None

    cur = (round(cfg["neutral"]["tilt_x_deg"], 3), round(cfg["neutral"]["tilt_y_deg"], 3), cfg["neutral"].get("twist_deg", 0))
    s_cur, fail_cur = score(R_n_of(*cur))
    print(f"current mount (tilt_x {cur[0]}, tilt_y {cur[1]}, twist {cur[2]}): gait fits up to x{s_cur:.2f}; "
          f"first failing checks {fail_cur}")

    # spin about the ring axis really changes nothing (sanity check of the reduction above)
    s_spin, _ = score(rot_z(math.radians(37)) @ R_n_of(*cur))
    print(f"  same, spun 37 deg about the ring axis: x{s_spin:.2f}  (should be identical)")

    grid = []
    rng = np.arange(-args.max_tilt, args.max_tilt + 1e-9, args.step)
    tws = np.arange(0, 120, 10.0)                          # the head is 3-fold symmetric: 0..120 covers all
    for tw in tws:
        for tx in rng:
            for ty in rng:
                if math.hypot(tx, ty) > args.max_tilt + 1e-9:
                    continue
                s, fail = score(R_n_of(tx, ty, tw))
                grid.append((tx, ty, tw, s))
    G = np.array(grid)
    print(f"scanned {len(G)} mounts; best gait scale x{G[:, 3].max():.2f}; "
          f"{(G[:, 3] >= 1).sum()} fit the whole gait, {(G[:, 3] >= 1.3).sum()} with >= 30% headroom")
    OUT.mkdir(exist_ok=True)
    np.save(OUT / "remount_scan.npy", G)

    # report: best overall, and the best that changes the current neutral the least
    R_cur = R_n_of(*cur)
    def change(row):
        return math.degrees(np.linalg.norm(rotvec(R_n_of(*row[:3]) @ R_cur.T)))
    rows = sorted(grid, key=lambda r: (-r[3], change(r)))
    print("\n best mounts (gait scale, then smallest change from today):")
    print("  scale  tilt_x  tilt_y  twist  change | ring axis in pelvis: down / forward / base twist")
    shown = 0
    for r in rows:
        R_n = R_n_of(*r[:3])
        R_PB = F_E.T @ R_n.T
        md = mount_angles(R_PB, hip.side)
        print(f"  x{r[3]:.2f}  {r[0]:6.1f}  {r[1]:6.1f}  {r[2]:5.0f}  {change(r):5.1f}  |  {md[0]:6.1f} {md[1]:7.1f} {md[2]:7.1f}")
        shown += 1
        if shown >= 12:
            break
    for need in (1.0, 1.2, 1.3, 1.5):
        cand = [r for r in grid if r[3] >= need]
        if cand:
            r = min(cand, key=change)
            md = mount_angles(F_E.T @ R_n_of(*r[:3]).T, hip.side)
            print(f"  smallest change reaching x{need:.1f}: tilt_x {r[0]:.1f} tilt_y {r[1]:.1f} twist {r[2]:.0f} "
                  f"(change {change(r):.1f} deg, scale x{r[3]:.2f}) -> mount down {md[0]:.1f}, forward {md[1]:.1f}, twist {md[2]:.1f}")
    json.dump({"current": {"neutral": cur, "scale": s_cur},
               "rows": [list(map(float, r)) + [change(r)] for r in rows[:200]]},
              open(OUT / "remount_scan_top.json", "w"), indent=1)


if __name__ == "__main__":
    main()

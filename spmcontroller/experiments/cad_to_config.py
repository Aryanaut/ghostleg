"""
cad_to_config.py - Derive the SPM geometry, mount and neutral for config.json
from an Onshape STEP export (the method used for v1 on 2026-11-17, made
repeatable).  Writes a NEW config file; never touches config.json.
Analysis tool, kept in experiments/ on purpose.

    python experiments/cad_to_config.py "bone-in SPM 2.step" --out config_v2.json

Method (HANDOFF section 2.1):
  * parts and joint axes: cad_axes.py; who is who: cad_collision.rig_layout()
    (rings / arms / middle links / femur group found by geometry, not names);
  * centre of rotation = least-squares point nearest the ring axis, the three
    arm (middle-joint) bores and the three femur-head bores;
  * base frame: origin = centre, z = ring axis pointing from the casing to the
    head, x = CAD x made perpendicular to z (ring angle 0);
  * w_i (middle joint) and v_i (head joint) point from the centre toward the
    bore on that part; alpha1 = angle(w, -z), alpha2 = angle(w, v);
  * effector frame: z_E = the femur head's joint-plane normal (toward the
    femur), x_E chosen so that the CAD pose has no twist; beta / eta of each
    v_i in that frame; branch per leg = the IK solution matching the CAD;
  * neutral = the CAD pose (assumption, as for v1);
  * pelvis frame from the femur (RIGHT leg): Y = knee centre -> head centre
    (mechanical axis), Z = lateral = widest direction of the distal 40 mm
    (knee), X = Y x Z; mount angles from that.  If a face is colour-marked as
    ground-parallel (cad_axes --colours), it is reported but not used yet.
  * gear ratio from the tooth counts in the gear names ("...108T" / "...18T").
The elbow / wrist collision limits are NOT computed here: run
    python experiments/cad_collision.py "<step>" limits --config <out>
afterwards (it writes them into the same file with --write).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import cad_axes as C                                              # noqa: E402
import cad_collision as K                                         # noqa: E402
import remount_study as S                                         # noqa: E402
from spm.kinematics import CoaxialSPM, SPMGeometry, rot_x, rot_y  # noqa: E402
from spm.anatomy import HipMapping                                # noqa: E402


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def derive(step: Path, base_cfg: dict, log=print) -> dict:
    E = C.load(step)
    parts = C.assembly(E)
    P = {p["nauo"]: p for p in parts}
    lay = K.rig_layout(step)
    name = lay["names"]
    c = np.array(lay["centre"])
    ax = {k: C.merge_axes(C.part_features(E, P[k])[0]) for k in P}
    big = lambda k: [g for g in ax[k] if 9.0 in g["radii"]]
    pts = K.load_points(step)                                      # dense surface samples, mm, CAD coords

    # ---- residual of the common centre (all joint axes should meet) -----------
    lines = []
    for L in lay["legs"]:
        g = big(L["arm"])[0]
        lines.append((g["p"], g["d"]))
    head = lay["femur"][0]
    for g in big(head):
        lines.append((g["p"], g["d"]))
    res = max(np.linalg.norm((c - p) - ((c - p) @ d) * d) for p, d in lines)
    log(f"centre of rotation (CAD mm) {np.round(c, 3)}; joint axes miss it by at most {res:.4f} mm")

    # ---- base frame ---------------------------------------------------------------
    ring_g = max(ax[lay["legs"][0]["ring"]], key=lambda g: g["n"])
    z = unit(ring_g["d"])
    casing = [k for k in lay["fixed"] if "casing" in name[k]]
    cas_ctr = np.mean(pts[casing[0]][1], axis=0) if casing else c - 50 * z
    if (c - cas_ctr) @ z < 0:
        z = -z                                                     # z: casing -> head
    x = np.array([1.0, 0, 0]) - z[0] * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0, 1.0, 0]) - z[1] * z
    x = unit(x)
    B = np.column_stack([x, np.cross(z, x), z])                    # base axes in CAD coords
    toB = lambda v: B.T @ v

    def toward(g, loc):                                            # axis direction pointing at a location
        d = unit(g["d"])
        return d if (loc - c) @ d > 0 else -d

    W, V, legs_out = [], [], []
    for i, L in enumerate(lay["legs"]):
        gw = big(L["arm"])[0]
        w = toB(toward(gw, gw["p"]))
        gv = [g for g in big(head) if abs(unit(g["d"]) @ unit(gw["d"])) < 0.9999
              and any(abs(unit(g["d"]) @ unit(h["d"])) > 0.9999 for h in big(L["mid"]))][0]
        v = toB(toward(gv, gv["p"]))
        W.append(w)
        V.append(v)
    W, V = np.array(W), np.array(V)
    a1 = [math.degrees(math.acos(np.clip(-w[2], -1, 1))) for w in W]
    a2 = [math.degrees(math.acos(np.clip(w @ v, -1, 1))) for w, v in zip(W, V)]
    th_cad = np.array([math.atan2(w[1], w[0]) for w in W])

    # ---- effector frame: z_E = normal of the plane through the v tips, toward the femur
    n = unit(np.cross(V[1] - V[0], V[2] - V[0]))
    fem = np.vstack([pts[k][1] for k in lay["femur"][1:] if k in pts and "key" not in name[k]]) - c
    if np.mean(fem @ B, axis=0) @ n < 0:
        n = -n
    tx = math.asin(-n[1])
    ty = math.atan2(n[0], n[2])
    Rn = rot_y(ty) @ rot_x(tx)
    VE = (Rn.T @ V.T).T
    beta = [math.degrees(math.acos(np.clip(v[2], -1, 1))) for v in VE]
    eta = [math.degrees(math.atan2(v[1], v[0])) for v in VE]

    # ---- branch per leg: the IK solution that reproduces the CAD ring angles --------
    best = None
    for br in [(a, b, cc) for a in (1, -1) for b in (1, -1) for cc in (1, -1)]:
        g = SPMGeometry(alpha1_deg=a1, alpha2_deg=a2, beta_deg=beta, eta_deg=eta, branch=list(br))
        th, _ = CoaxialSPM(g).ik(Rn)
        err = np.abs(np.degrees(np.angle(np.exp(1j * (th - th_cad))))).max()
        if best is None or err < best[0]:
            best = (err, br)
    log(f"alpha1 {np.round(a1, 3)}  alpha2 {np.round(a2, 3)}  beta {np.round(beta, 3)}  eta {np.round(eta, 3)}")
    log(f"branch {best[1]}: IK reproduces the CAD ring angles to {best[0]:.4f} deg; "
        f"CAD pose tilted {math.degrees(math.acos(n[2])):.2f} deg from the symmetric home")

    # ---- femur -> pelvis frame (RIGHT leg) -----------------------------------------
    F = fem @ B                                                    # base frame, centred
    far = F[np.argmax(np.linalg.norm(F, axis=1))]
    a0 = unit(far)
    proj = F @ a0
    D = F[proj > proj.max() - 40]
    knee = D.mean(0)
    Y = -unit(knee)
    Q = D - knee
    Q -= np.outer(Q @ Y, Y)
    ml = unit(np.linalg.svd(Q, full_matrices=False)[2][0])
    shaft = F[(F @ -Y > 120) & (F @ -Y < 300)]
    sc = shaft.mean(0)
    sd = unit(np.linalg.svd(shaft - sc)[2][0])
    sd = sd if sd @ -Y > 0 else -sd
    off = -sc - ((-sc) @ sd) * sd
    med = off - (off @ Y) * Y
    if ml @ med > 0:
        ml = -ml                                                   # lateral = away from the head offset
    # A colour-marked ground-parallel face (cad_axes --colours) gives TRUE vertical:
    # then pelvis Y = up from that face instead of the femur's mechanical axis
    # (sign chosen so the femur hangs down), and the femur's own lean is reported.
    ground = None
    try:
        up_cad, face = C.ground_up(E, parts, head_to_knee=B @ knee)
        ground = toB(up_cad)
        lean = math.degrees(math.acos(np.clip(-Y @ -ground, -1, 1)))
        log(f"ground face: {name[face['nauo']]} (outward normal CAD {np.round(face['normal'], 4)}); "
            f"femur mechanical axis is {lean:.2f} deg from vertical in the CAD pose")
        Y = ground
    except ValueError as exc:
        log(f"no ground face used ({exc}); vertical = femur mechanical axis")
    # femur's own frame at the CAD pose (base coords): Yf up its mechanical axis,
    # Zf lateral (knee axis), Xf anterior
    Yf = -unit(knee)
    Zf = unit(ml - (ml @ Yf) * Yf)
    Ff_B = np.column_stack([np.cross(Yf, Zf), Yf, Zf])
    Zl = unit(ml - (ml @ Y) * Y)
    X = np.cross(Y, Zl)
    R_PB = np.vstack([X, Y, Zl])                                   # base axes in pelvis frame (columns)
    R_neutral = Rn                                                 # default (no ground face): CAD pose = neutral
    if ground is not None:
        # With true vertical known, NEUTRAL = femur standing: mechanical axis vertical,
        # knee axis horizontal and facing as in the CAD.  The leg is rigid on the
        # effector, so neutral is the CAD effector pose rotated by the rotation that
        # takes the CAD femur frame onto the pelvis frame:  R_n = P_B Ff_B^T R_cad.
        P_B = R_PB.T                                               # pelvis axes in base coords (columns)
        h0 = HipMapping({"mount": {"side": "right"}})
        cad_anat = h0.anat_angles(R_PB @ Ff_B)                     # where the CAD leg is, in hip angles
        log(f"the CAD file shows the leg at flexion {cad_anat[0]:+.1f}, adduction {cad_anat[1]:+.1f}, "
            f"int. rotation {cad_anat[2]:+.1f} deg (relative to standing neutral)")
        R_neutral = P_B @ Ff_B.T @ Rn
    mount = S.mount_angles(R_PB, 1)
    log(f"femur: knee centre {np.linalg.norm(knee):.1f} mm from the head; mount tilt_down {mount[0]:.2f}, "
        f"forward {mount[1]:.2f}, base_twist {mount[2]:.2f}")

    # ---- gear ratio from names -------------------------------------------------------
    teeth = lambda k: int(re.search(r"(\d+)T", name[k]).group(1)) if re.search(r"(\d+)T", name[k]) else None
    ring_t = teeth(lay["legs"][0]["ring"])
    pin_t = [teeth(k) for k in lay["fixed"] if teeth(k)]
    ratio = ring_t / pin_t[0] if ring_t and pin_t else base_cfg["control"]["gear_ratio"]

    # ---- assemble the new config ---------------------------------------------------------
    cfg = json.loads(json.dumps(base_cfg))
    cfg["geometry"].update({"_source": f"cad_to_config.py on {step.name}",
                            "alpha1_deg": [round(x, 3) for x in a1], "alpha2_deg": [round(x, 3) for x in a2],
                            "beta_deg": [round(x, 3) for x in beta], "eta_deg": [round(x, 3) for x in eta],
                            "branch": list(best[1])})
    cfg["mount"].update({"_source": f"cad_to_config.py on {step.name}: CAD pose = neutral, RIGHT femur",
                         "side": "right", "tilt_down_deg": round(mount[0], 2), "forward_deg": round(mount[1], 2),
                         "base_twist_deg": round(mount[2], 2)})
    # R_neutral = Rz(twist) Ry(tilt_y) Rx(tilt_x)  (anatomy.py)
    Rq = R_neutral
    cfg["neutral"].update({"_source": f"cad_to_config.py on {step.name}: " +
                           ("femur standing (vertical from the marked ground face)" if ground is not None
                            else "the CAD pose"),
                           "tilt_x_deg": round(math.degrees(math.atan2(Rq[2, 1], Rq[2, 2])), 4),
                           "tilt_y_deg": round(math.degrees(math.asin(np.clip(-Rq[2, 0], -1, 1))), 4),
                           "twist_deg": round(math.degrees(math.atan2(Rq[1, 0], Rq[0, 0])), 4)})
    cfg["femur"]["mech_length_m"] = round(float(np.linalg.norm(knee)) / 1000, 3)
    cfg["visual"]["r_prox_m"] = round(float(np.mean([np.linalg.norm(big(L["arm"])[0]["p"] - c)
                                                       for L in lay["legs"]])) / 1000, 3)
    cfg["visual"]["r_dist_m"] = round(float(np.mean([np.linalg.norm(g["p"] - c) for g in big(head)])) / 1000, 3)
    cfg["control"]["gear_ratio"] = ratio
    # collision limits belong to the OLD CAD: keep the reference pose, disable the
    # fold intervals until cad_collision.py has measured this CAD
    jl = cfg["limits"].setdefault("joint_limits", {})
    jl.update({"_source": f"NOT YET MEASURED for {step.name}: run cad_collision.py limits",
               "ref_R_BE": np.round(Rn, 9).tolist(),
               "elbow_deg": [[-180, 180]] * 3, "wrist_deg": [[-180, 180]] * 3})
    # sanity: neutral maps to 0/0/0, and the CAD pose maps back to where the CAD leg is
    h = HipMapping(cfg)
    f, ad, r = h.anat_angles(h.anat_from_effector(h.R_neutral))
    assert max(abs(f), abs(ad), abs(r)) < 1e-3, (f, ad, r)
    f, ad, r = h.anat_angles(h.anat_from_effector(Rn))
    log(f"check: the CAD effector pose reads flex {f:+.2f} add {ad:+.2f} rot {r:+.2f} in the new config; "
        f"neutral is {math.degrees(np.linalg.norm(__import__('spm.kinematics', fromlist=['rotvec']).rotvec(h.R_neutral @ Rn.T))):.2f} deg from it")
    th_n, m_n = CoaxialSPM(SPMGeometry.from_dict(cfg["geometry"])).ik(h.R_neutral)
    log(f"neutral reachable: {bool(np.all(np.isfinite(th_n)))} (leg margins {np.round(m_n, 3)})")
    log(f"gear ratio {ratio:g} ({ring_t}T / {pin_t[0] if pin_t else '?'}T); r_prox {cfg['visual']['r_prox_m']} m, "
        f"r_dist {cfg['visual']['r_dist_m']} m; marked ground faces: {len(C.marked_faces(E, parts))}")
    return cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--base", default=str(ROOT / "config.json"), help="config to copy everything else from")
    ap.add_argument("--out", required=True, help="new config file to write (never config.json)")
    args = ap.parse_args()
    out = Path(args.out)
    if out.resolve() == (ROOT / "config.json").resolve():
        raise SystemExit("refusing to overwrite config.json; write a new file and switch deliberately")
    cfg = derive(Path(args.step), json.loads(Path(args.base).read_text()))
    out.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

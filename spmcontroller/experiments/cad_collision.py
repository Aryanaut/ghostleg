"""
cad_collision.py - Link-collision analysis of the SPM straight from the CAD
(Onshape STEP export), with numpy + scipy only (no CAD kernel needed).
Analysis tool, kept in experiments/ on purpose.

    python experiments/cad_collision.py "bone-in SPM.step" baseline   # clearances at the CAD pose
    python experiments/cad_collision.py "bone-in SPM.step" limits     # -> config.json limits (~10 min)

Pipeline
--------
1. SURFACE POINTS.  Every face of every part is covered with points about
   H_MM apart, in the part's assembly position:
     * boundary edges (LINE / CIRCLE / ELLIPSE / (rational) B-spline) are
       sampled between their two vertices, in loop order;
     * PLANE faces: a grid on the plane, kept if inside the face's boundary
       loops (even-odd rule, so holes are respected);
     * CYLINDER / CONE / SPHERE / TORUS faces: a grid in the surface's own
       (angle, height/angle) coordinates, trimmed the same way after unwrapping
       the angle at the face's largest angular gap (or the full band when the
       face goes all the way round);
     * (rational) B-SPLINE faces (the bone, the ball): the whole patch domain.
   Two surfaces that touch or cross then always have points within ~H_MM of
   each other, so 'clearance' below is accurate to about +-H_MM.
   Cached in experiments/cad_cache/ (keyed by STEP file size + mtime).

2. POSE MODEL.  For an effector orientation R (base frame = CAD axes moved
   to the centre of rotation; see HANDOFF 2.1):
     * casing, pinions, positioning key: fixed;
     * ring i and arm i: rotate about the ring axis by (theta_i - theta_i_CAD);
     * middle arm i: the rotation taking (w_i, v_i) at the CAD pose to
       (w_i, v_i) now (alpha2 = 90 deg, so this frame is well defined);
     * femur head, upper and lower femur: R @ R_CAD^T.
   theta comes from the controller's own IK with config.json, so the check
   uses exactly the kinematics that drive the motors.

3. CLEARANCE.  For each pair of parts in different motion groups, the
   minimum point-to-point distance (KD-tree, scipy), culled by 20 mm clusters.

4. LIMITS (section 4 below): pairs are sorted by how they move relative to
   each other, which turns almost all of them into 1-D tables -> a minimum
   ring separation and elbow / wrist fold intervals that the controller can
   check every tick.  The few pose-dependent pairs left are swept over the
   whole range the new limits allow, to prove they keep their distance.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import cad_axes as C                                                  # noqa: E402

H_MM = 1.5            # surface sampling spacing (mm)
H = H_MM / 1000       # same, in the STEP's metres


# ===========================================================================
# 1a. NURBS evaluation
# ===========================================================================

def basis(U, p, t):
    """B-spline basis functions N_{i,p}(t) for all i, vectorised over t.
    U = full knot vector, p = degree.  Returns (len(t), len(U)-p-1)."""
    U = np.asarray(U, float)
    t = np.asarray(t, float)
    m = len(U) - 1
    N = np.zeros((len(t), m))
    for i in range(m):                                    # degree 0
        N[:, i] = (U[i] <= t) & (t < U[i + 1])
    # t == last knot: belongs to the last non-empty span
    last = max(i for i in range(m) if U[i] < U[i + 1])
    N[t >= U[-1], :] = 0
    N[t >= U[-1], last] = 1
    for k in range(1, p + 1):                             # Cox - de Boor
        Nk = np.zeros((len(t), m - k))
        for i in range(m - k):
            d1 = U[i + k] - U[i]
            d2 = U[i + k + 1] - U[i + 1]
            a = (t - U[i]) / d1 * N[:, i] if d1 > 0 else 0
            b = (U[i + k + 1] - t) / d2 * N[:, i + 1] if d2 > 0 else 0
            Nk[:, i] = a + b
        N = Nk
    return N


def knots_full(mults, knots):
    return np.repeat(np.asarray(knots, float), np.asarray(mults, int))


def bspline_curve_data(E, eid):
    """(degree, ctrl[n,3], weights[n], U) for a simple or rational B-spline curve."""
    t, a = E[eid]
    if t == "B_SPLINE_CURVE_WITH_KNOTS":
        p, cps, mults, kn = int(a[1]), a[2], a[6], a[7]
        w = None
    elif t == "COMPLEX":
        bc, bk = a["B_SPLINE_CURVE"], a["B_SPLINE_CURVE_WITH_KNOTS"]
        p, cps, mults, kn = int(bc[0]), bc[1], bk[0], bk[1]
        w = a.get("RATIONAL_B_SPLINE_CURVE", [None])[0]
    else:
        raise ValueError(t)
    P = np.array([C.point(E, r) for r in cps])
    W = np.ones(len(P)) if w is None else np.array(w, float)
    return p, P, W, knots_full(mults, kn)


def eval_curve(p, P, W, U, t):
    N = basis(U, p, t) * W
    return (N @ P) / N.sum(1, keepdims=True)


def bspline_surface_data(E, eid):
    t, a = E[eid]
    if t == "B_SPLINE_SURFACE_WITH_KNOTS":
        pu, pv, cps = int(a[1]), int(a[2]), a[3]
        um, vm, uk, vk = a[8], a[9], a[10], a[11]
        w = None
    elif t == "COMPLEX":
        bs, bk = a["B_SPLINE_SURFACE"], a["B_SPLINE_SURFACE_WITH_KNOTS"]
        pu, pv, cps = int(bs[0]), int(bs[1]), bs[2]
        um, vm, uk, vk = bk[0], bk[1], bk[2], bk[3]
        w = a.get("RATIONAL_B_SPLINE_SURFACE", [None])[0]
    else:
        raise ValueError(t)
    P = np.array([[C.point(E, r) for r in row] for row in cps])
    W = np.ones(P.shape[:2]) if w is None else np.array(w, float)
    return pu, pv, P, W, knots_full(um, uk), knots_full(vm, vk)


def sample_bspline_surface(E, eid, loops=None):
    """Points on a (rational) B-spline patch.  If the face's boundary loops
    are given, the patch is TRIMMED to them: each boundary point is mapped to
    the (u, v) of the nearest surface sample, and grid points are kept if
    they are inside those (u, v) loops or within one spacing of the boundary
    in 3-D (so samples on the patch border are not lost to the in/out test)."""
    pu, pv, P, W, U, V = bspline_surface_data(E, eid)
    # number of samples from the control net's extent in each direction
    lu = max(np.linalg.norm(np.diff(P[:, j], axis=0), axis=1).sum() for j in range(P.shape[1]))
    lv = max(np.linalg.norm(np.diff(P[i], axis=0), axis=1).sum() for i in range(P.shape[0]))
    nu, nv = max(2, int(math.ceil(lu / H)) + 1), max(2, int(math.ceil(lv / H)) + 1)
    tu = np.linspace(U[pu], U[-pu - 1], nu)
    tv = np.linspace(V[pv], V[-pv - 1], nv)
    Nu, Nv = basis(U, pu, tu), basis(V, pv, tv)
    num = np.einsum("ai,bj,ij,ijk->abk", Nu, Nv, W, P)
    den = np.einsum("ai,bj,ij->ab", Nu, Nv, W)
    S = (num / den[..., None]).reshape(-1, 3)
    if not loops:
        return S
    # (u, v) grid index of every sample, normalised to [0, 1]
    gu, gv = np.meshgrid(np.linspace(0, 1, nu), np.linspace(0, 1, nv), indexing="ij")
    UV = np.column_stack([gu.ravel(), gv.ravel()])
    tree = cKDTree(S)
    uv_loops = [UV[tree.query(L)[1]] for L in loops]
    keep = inside(uv_loops, UV)
    keep |= cKDTree(np.vstack(loops)).query(S)[0] < H      # border band
    return S[keep]


# ===========================================================================
# 1b. edges and loops
# ===========================================================================

def sample_edge(E, edge_curve_id):
    """Points along an EDGE_CURVE from its start vertex to its end vertex."""
    _, a = E[edge_curve_id]
    v1 = C.point(E, E[C.R(a[1])][1][1])
    v2 = C.point(E, E[C.R(a[2])][1][1])
    gid = C.R(a[3])
    same = a[4] == ".T."
    gt, ga = E[gid]
    if gt == "LINE":
        n = max(2, int(np.linalg.norm(v2 - v1) / H) + 1)
        return v1 + np.linspace(0, 1, n)[:, None] * (v2 - v1)
    if gt in ("CIRCLE", "ELLIPSE"):
        M = C.placement(E, ga[1])
        r1 = ga[2]
        r2 = ga[3] if gt == "ELLIPSE" else ga[2]

        def ang(q):
            d = q - M[:3, 3]
            return math.atan2((d @ M[:3, 1]) / r2, (d @ M[:3, 0]) / r1)
        a1, a2 = ang(v1), ang(v2)
        if a[1] == a[2]:                                   # same vertex entity: a full circle/ellipse
            sweep = 2 * math.pi
        else:
            sweep = (a2 - a1) % (2 * math.pi) if same else -((a1 - a2) % (2 * math.pi))
            # Distinct vertices but ~a full turn = a TINY arc whose end
            # parameter rounded to just behind its start (seen in this CAD: a
            # 0.01 mm arc of an ellipse with a 381 mm major radius, which would
            # otherwise be sampled as a 760 mm long phantom ellipse).
            if abs(sweep) > 2 * math.pi - 1e-3:
                sweep -= math.copysign(2 * math.pi, sweep)
        n = max(3, int(abs(sweep) * max(r1, r2) / H) + 1)
        tt = a1 + np.linspace(0, sweep, n)
        return (M[:3, 3] + np.outer(r1 * np.cos(tt), M[:3, 0]) + np.outer(r2 * np.sin(tt), M[:3, 1]))
    # (rational) B-spline: sample all of it, keep the stretch between the vertices
    p, P, W, U = bspline_curve_data(E, gid)
    L = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
    t = np.linspace(U[p], U[-p - 1], max(4, int(L / H) * 2 + 1))
    X = eval_curve(p, P, W, U, t)
    i1 = int(np.argmin(np.linalg.norm(X - v1, axis=1)))
    i2 = int(np.argmin(np.linalg.norm(X - v2, axis=1)))
    if a[1] == a[2]:                                       # closed edge: the whole curve
        return X if same else X[::-1]
    if i1 == i2:                                           # very short edge
        return np.array([v1, v2])
    seg = X[min(i1, i2):max(i1, i2) + 1]
    return seg if i1 < i2 else seg[::-1]


def face_loops(E, face_id):
    """Boundary loops of a face, each an ordered (n,3) polyline."""
    loops = []
    for b in C.refs_in(E[face_id][1][1]):
        bt, ba = E[b]
        lt, la = E[C.R(ba[1])]
        if lt != "EDGE_LOOP":
            continue                                       # VERTEX_LOOP: a degenerate point
        pts = []
        for oe in C.refs_in(la[1]):
            oa = E[oe][1]
            seg = sample_edge(E, C.R(oa[3]))
            pts.append(seg if oa[4] == ".T." else seg[::-1])
        loops.append(np.vstack(pts))
    return loops


def inside(poly_list, Q):
    """Even-odd point-in-polygon for 2-D points Q against several closed loops."""
    ins = np.zeros(len(Q), bool)
    for poly in poly_list:
        A = poly
        B = np.roll(poly, -1, axis=0)
        for s in range(0, len(Q), 4000):                  # chunk: memory
            q = Q[s:s + 4000]
            ya, yb = A[None, :, 1], B[None, :, 1]
            cond = (ya > q[:, None, 1]) != (yb > q[:, None, 1])
            with np.errstate(divide="ignore", invalid="ignore"):
                xint = A[None, :, 0] + (q[:, None, 1] - ya) * (B[None, :, 0] - A[None, :, 0]) / (yb - ya)
            ins[s:s + 4000] ^= (cond & (q[:, None, 0] < xint)).sum(1) % 2 == 1
    return ins


def unwrap_angles(ang, step):
    """Angles -> (offset, span, full).  If the boundary goes all the way round
    (largest gap ~ one sample step) the face is a full band."""
    a = np.sort(np.mod(ang, 2 * math.pi))
    gaps = np.diff(np.concatenate([a, [a[0] + 2 * math.pi]]))
    k = int(np.argmax(gaps))
    if gaps[k] < 4 * step:
        return 0.0, 2 * math.pi, True
    start = a[(k + 1) % len(a)]                             # first angle after the gap
    return start, 2 * math.pi - gaps[k], False


def grid2d(lo, hi, step):
    xs = np.arange(lo[0], hi[0] + step[0] * 0.5, step[0])
    ys = np.arange(lo[1], hi[1] + step[1] * 0.5, step[1])
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack([X.ravel(), Y.ravel()])


def sample_face(E, face_id):
    """Points on one ADVANCED_FACE (part coordinates, metres)."""
    srf = C.R(E[face_id][1][2])
    st, sa = E[srf]
    loops = face_loops(E, face_id)
    B = np.vstack(loops) if loops else np.zeros((0, 3))
    if st in ("B_SPLINE_SURFACE_WITH_KNOTS", "COMPLEX"):
        return np.vstack([sample_bspline_surface(E, srf, loops), B])
    M = C.placement(E, sa[1])
    o, x, y, z = M[:3, 3], M[:3, 0], M[:3, 1], M[:3, 2]

    def local(P):
        d = P - o
        return d @ x, d @ y, d @ z

    if st == "PLANE":
        lps = [np.column_stack(local(L)[:2]) for L in loops]
        allp = np.vstack(lps)
        G = grid2d(allp.min(0), allp.max(0), (H, H))
        G = G[inside(lps, G)]
        return np.vstack([o + np.outer(G[:, 0], x) + np.outer(G[:, 1], y), B])

    if st in ("CYLINDRICAL_SURFACE", "CONICAL_SURFACE"):
        r0 = sa[2]
        tan_a = math.tan(sa[3]) if st == "CONICAL_SURFACE" else 0.0
        bx, by, bz = local(B)
        u = np.arctan2(by, bx)
        rmax = max(r0 + tan_a * bz.max(), r0 + tan_a * bz.min(), 1e-4)
        off, span, full = unwrap_angles(u, H / rmax)
        du = H / rmax
        if full:
            G = grid2d((0, bz.min()), (2 * math.pi - du, bz.max()), (du, H))
        else:
            lps = [np.column_stack([np.mod(np.arctan2(local(L)[1], local(L)[0]) - off, 2 * math.pi) * rmax,
                                    local(L)[2]]) for L in loops]
            G = grid2d((0, bz.min()), (span * rmax, bz.max()), (H, H))
            G = G[inside(lps, G)]
            G[:, 0] = G[:, 0] / rmax
        uu, vv = G[:, 0] + off, G[:, 1]
        rr = r0 + tan_a * vv
        return np.vstack([o + np.outer(rr * np.cos(uu), x) + np.outer(rr * np.sin(uu), y) + np.outer(vv, z), B])

    if st == "SPHERICAL_SURFACE":
        r = sa[2]
        bx, by, bz = local(B)
        lon = np.arctan2(by, bx)
        lat = np.arcsin(np.clip(bz / r, -1, 1))
        off, span, full = unwrap_angles(lon, H / r)
        G = grid2d((0, lat.min()), (span, lat.max()), (H / r, H / r))
        uu, vv = G[:, 0] + off, G[:, 1]
        return np.vstack([o + r * (np.outer(np.cos(vv) * np.cos(uu), x) + np.outer(np.cos(vv) * np.sin(uu), y)
                                   + np.outer(np.sin(vv), z)), B])

    if st == "TOROIDAL_SURFACE":
        Rm, rm = sa[2], sa[3]
        bx, by, bz = local(B)
        u = np.arctan2(by, bx)
        v = np.arctan2(bz, np.hypot(bx, by) - Rm)
        ou, su, _ = unwrap_angles(u, H / (Rm + rm))
        ov, sv, _ = unwrap_angles(v, H / rm)
        G = grid2d((0, 0), (su, sv), (H / (Rm + rm), H / rm))
        uu, vv = G[:, 0] + ou, G[:, 1] + ov
        rho = Rm + rm * np.cos(vv)
        return np.vstack([o + np.outer(rho * np.cos(uu), x) + np.outer(rho * np.sin(uu), y)
                          + np.outer(rm * np.sin(vv), z), B])

    return B                                               # unknown surface type: edges only


def sample_part(E, part):
    """All surface points of one assembly part, assembly coordinates, mm."""
    pts = []
    for s in part["solids"]:
        shell = C.R(E[s][1][1])
        for f in C.refs_in(E[shell][1][1]):
            if E[f][0] == "ADVANCED_FACE":
                pts.append(sample_face(E, f))
    X = np.vstack(pts) * 1000
    T = part["T"]
    return (T[:3, :3] @ X.T).T + T[:3, 3] * 1000


def load_points(step: Path):
    """{nauo: (name, points_mm)} for every part, cached."""
    cache = HERE / "cad_cache" / f"{step.stem}_{step.stat().st_size}_{int(step.stat().st_mtime)}_h{H_MM}.npz"
    if cache.exists():
        z = np.load(cache, allow_pickle=True)
        names = json.loads(str(z["names"]))
        return {k: (names[k], z[k]) for k in names}
    t0 = time.time()
    E = C.load(step)
    out = {}
    for prt in C.assembly(E):
        out[prt["nauo"]] = (prt["name"], sample_part(E, prt))
        print(f"  sampled {prt['nauo']:7s} {prt['name']:20s} {len(out[prt['nauo']][1]):7d} pts", flush=True)
    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(cache, names=json.dumps({k: v[0] for k, v in out.items()}),
                        **{k: v[1] for k, v in out.items()})
    print(f"  surface sampling took {time.time() - t0:.0f} s -> {cache.name}")
    return out


# ===========================================================================
# 2. Which part is which: centre of rotation, rings, arms, middle arms
# ===========================================================================

def rig_layout(step: Path) -> dict:
    """Identify the moving groups from the CAD's joint axes (cached as json):
       centre   point all joint axes pass through (mm, CAD coords)
       legs[i]  {ring, arm, mid}: NAUO ids for leg i (leg i = 'arm{i+1}')
       femur    NAUO ids that move with the effector (femur head, bone, key)
       fixed    everything else (casing, pinions)"""
    cache = HERE / "cad_cache" / f"{step.stem}_{step.stat().st_size}_{int(step.stat().st_mtime)}_layout.json"
    if cache.exists():
        return json.loads(cache.read_text())
    E = C.load(step)
    parts = C.assembly(E)
    name = {p["nauo"]: p["name"] for p in parts}
    ax = {p["nauo"]: C.merge_axes(C.part_features(E, p)[0]) for p in parts}
    by = lambda n: [k for k, v in name.items() if v == n]
    big = lambda k: [g for g in ax[k] if 9.0 in g["radii"]]          # the 18 mm joint bores
    par = lambda a, b: abs(float(a["d"] @ b["d"])) > 0.9999

    def same_line(a, b):
        off = b["p"] - a["p"]
        return par(a, b) and np.linalg.norm(off - (off @ a["d"]) * a["d"]) < 0.2

    head = by("femur head")[0]
    # Gears are recognised by GEOMETRY, not by name (v1 rings were "72T", v2 "108T"):
    # a part whose main axis carries many coaxial cylinders (the teeth).  The
    # three RINGS share one axis; the pinions each sit on their own.
    gears = {k: max(ax[k], key=lambda g: g["n"]) for k in name if ax[k] and max(g["n"] for g in ax[k]) >= 30}
    rings = [k for k, g in gears.items() if sum(same_line(g, h) for h in gears.values()) >= 3]
    ring_axis = gears[rings[0]]
    arms = [by(f"arm{i + 1}")[0] for i in range(3)]
    # middle (distal) links: any other part with 18 mm bores (v1 "middle arm", v2 "Part 11/12/13")
    mids = [k for k in name if k not in arms + rings + [head] and len(big(k)) >= 2]
    lines = [(ring_axis["p"], ring_axis["d"])]
    legs = []
    for i, arm in enumerate(arms):
        w = big(arm)[0]
        lines.append((w["p"], w["d"]))
        mid = [m for m in mids if any(par(w, g) for g in ax[m])][0]
        pins = [g for g in ax[arm] if par(g, ring_axis) and not same_line(g, ring_axis)]   # pins into the ring
        ring = [r for r in rings if any(par(h, ring_axis) and not same_line(h, ring_axis)
                                        and np.linalg.norm(np.cross(g["p"] - h["p"], ring_axis["d"])) < 0.1
                                        for g in pins for h in ax[r])][0]
        legs.append({"ring": ring, "arm": arm, "mid": mid})
    for g in big(head):
        lines.append((g["p"], g["d"]))
    centre = C.closest_point_to_lines(lines)
    femur = [head] + by("upper femur") + by("lower femur") + by("positioning key")
    used = {x for L in legs for x in L.values()} | set(femur)
    out = {"centre": centre.tolist(), "legs": legs, "femur": femur,
           "fixed": [k for k in name if k not in used], "names": name}
    cache.parent.mkdir(exist_ok=True)
    cache.write_text(json.dumps(out, indent=1))
    return out


# ===========================================================================
# 3. Pose model + clearance
# ===========================================================================

def rot_about_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class Rig:
    """Places every CAD part for a commanded effector orientation and
    measures part-to-part clearances.  All coordinates: mm, base frame
    (= CAD axes with the origin moved to the centre of rotation)."""

    CLUSTER_MM = 20.0          # spatial chunk size for fast culling
    REACH_MM = 15.0            # clearances above this are reported as REACH_MM

    def __init__(self, step: Path, config: Path = ROOT / "config.json"):
        from spm.anatomy import HipMapping
        from spm.kinematics import CoaxialSPM, SPMGeometry
        self.config_path = Path(config)
        self.cfg = json.loads(Path(config).read_text())
        self.hip = HipMapping(self.cfg)
        self.spm = CoaxialSPM(SPMGeometry.from_dict(self.cfg["geometry"]))
        self.lay = rig_layout(step)
        self.names = self.lay["names"]
        c = np.array(self.lay["centre"])
        pts = load_points(step)
        self.P = {k: v[1] - c for k, v in pts.items()}          # part points at the CAD pose, centred
        # Reference pose = the pose the CAD assembly is saved in.  Stored in
        # limits.joint_limits.ref_R_BE once known; before that, the config's
        # neutral was set FROM the CAD pose, so it is the same thing.
        jl = self.cfg.get("limits", {}).get("joint_limits", {})
        self.R0 = np.array(jl["ref_R_BE"], float) if "ref_R_BE" in jl else self.hip.R_neutral.copy()
        self.th0, _ = self.spm.ik(self.R0)
        self.W0 = self.spm.w_world(self.th0)
        self.V0 = self.spm.v_world(self.R0)
        self.group = {}                                          # nauo -> motion group
        for k in self.lay["fixed"]:
            self.group[k] = "fixed"
        for k in self.lay["femur"]:
            self.group[k] = "femur"
        for i, L in enumerate(self.lay["legs"]):
            self.group[L["ring"]] = self.group[L["arm"]] = f"leg{i}"
            self.group[L["mid"]] = f"mid{i}"
        # KD-trees and spatial clusters in each part's own (CAD-pose) frame
        self.tree = {k: cKDTree(X) for k, X in self.P.items()}
        self.clusters = {}
        for k, X in self.P.items():
            key = np.floor(X / self.CLUSTER_MM).astype(int)
            _, inv = np.unique(key, axis=0, return_inverse=True)
            inv = inv.ravel()
            cl = []
            for j in range(inv.max() + 1):
                idx = np.nonzero(inv == j)[0]
                ctr = X[idx].mean(0)
                cl.append((ctr, float(np.linalg.norm(X[idx] - ctr, axis=1).max()), idx))
            self.clusters[k] = (np.array([q[0] for q in cl]), np.array([q[1] for q in cl]), [q[2] for q in cl])
        # only parts in DIFFERENT motion groups can move relative to each other
        self.pairs = [(a, b) for a, b in itertools.combinations(sorted(self.P), 2)
                      if self.group[a] != self.group[b]]

    # ---- placement --------------------------------------------------------
    def rotations(self, R):
        """Rotation (about the centre) of every motion group for effector
        orientation R, or None if the pose is unreachable."""
        th, margin = self.spm.ik(R, self.th0)
        if not np.all(np.isfinite(th)):
            return None
        W = self.spm.w_world(th)
        V = self.spm.v_world(R)
        rot = {"fixed": np.eye(3), "femur": R @ self.R0.T}
        for i in range(3):
            rot[f"leg{i}"] = rot_about_z(th[i] - self.th0[i])       # ring + arm turn about the ring axis
            # middle arm: the rotation taking its two joint axes (w, v) from the CAD pose to now
            F0 = np.column_stack([self.W0[i], self.V0[i], np.cross(self.W0[i], self.V0[i])])
            F1 = np.column_stack([W[i], V[i], np.cross(W[i], V[i])])
            rot[f"mid{i}"] = F1 @ F0.T
        return rot

    # ---- clearance between two parts -------------------------------------------
    def pair_clearance(self, a, b, Ra, Rb, reach=None):
        """Min distance (mm) between parts a and b placed by rotations Ra, Rb,
        capped at `reach`."""
        reach = self.REACH_MM if reach is None else reach
        Rab = Ra.T @ Rb                                          # b's points into a's frame
        ca, ra, _ = self.clusters[a]
        cb, rb, idxb = self.clusters[b]
        cbt = cb @ Rab.T
        # cluster-sphere culling: only clusters of b that can be within reach of a
        D = np.linalg.norm(cbt[:, None, :] - ca[None, :, :], axis=2) - rb[:, None] - ra[None, :]
        cand = np.nonzero(D.min(1) < reach)[0]
        if len(cand) == 0:
            return reach
        idx = np.concatenate([idxb[j] for j in cand])
        d, _ = self.tree[a].query(self.P[b][idx] @ Rab.T, distance_upper_bound=reach)
        return float(min(reach, d.min()))

    def clearances(self, R, pairs=None):
        """{(a, b): clearance mm} for every checked pair, or None if unreachable."""
        rot = self.rotations(R)
        if rot is None:
            return None
        return {(a, b): self.pair_clearance(a, b, rot[self.group[a]], rot[self.group[b]])
                for a, b in (pairs or self.pairs)}

    def label(self, k):
        g = self.group[k]
        return self.names[k] + (f" [{g}]" if g.startswith(("leg", "mid")) else "")



# ===========================================================================
# 4. From clearances to controller limits
# ===========================================================================
#
# Pairs are sorted by HOW their parts move relative to each other:
#   * both turn about the ring axis z (rings, arms, casing, pinions): the
#     clearance depends on ONE angle, the relative ring rotation -> 1-D table;
#   * arm_i / ring_i vs middle arm_i: one rotation about the elbow axis w_i
#     -> 1-D table of the ELBOW angle;
#   * middle arm_i vs femur head / bone: one rotation about the head joint
#     axis v_i -> 1-D table of the WRIST angle;
#   * everything else ("general": middle arm vs another leg, femur vs arms /
#     rings / casing, ...): depends on the whole pose -> checked on a sweep.
# A 1-D table is turned into a free interval around the CAD pose.  Pairs whose
# clearance never really closes (gear mesh, bearings, stacked rings, an arm
# running past the casing: min > 0.05 mm and < 1 mm at the CAD pose) are
# RUNNING CONTACTS by design and are skipped.

TOUCH_MM = 0.3        # a varying pair counts as colliding below this
GENERAL_MIN_MM = 2.0  # the general pairs must keep at least this in the swept range


def _table(rig, parts_a, parts_b, rot_b, phis):
    res = np.full(len(phis), 99.0)
    used = []
    for a in parts_a:
        for b in parts_b:
            cl = np.array([rig.pair_clearance(a, b, np.eye(3), rot_b(p)) for p in phis])
            i0 = int(np.argmin(np.abs(phis)))
            if cl.min() > 0.05 and cl[i0] < 1.0:
                continue                                   # running contact by design
            used.append([rig.label(a), rig.label(b), round(float(cl.min()), 2)])
            res = np.minimum(res, cl)
    return res, used


def _free(phis, cl):
    ok = cl >= TOUCH_MM
    i0 = int(np.argmin(np.abs(phis)))
    if not ok[i0]:
        raise RuntimeError("the CAD pose itself is in contact")
    lo = hi = i0
    while hi + 1 < len(phis) and ok[hi + 1]:
        hi += 1
    while lo - 1 >= 0 and ok[lo - 1]:
        lo -= 1
    return float(phis[lo]), float(phis[hi])


def compute_limits(rig: Rig, log=print) -> dict:
    from spm.kinematics import axis_angle
    L = rig.lay["legs"]
    out = {"ring_contact_sep_deg": [], "elbow_deg": [], "wrist_deg": [], "pairs": {}}
    # (a) ring separation: rotate leg j's ring+arm relative to leg i's
    phis = np.arange(-359.5, 360, 0.5)
    for i, j in ((0, 1), (1, 2), (0, 2)):
        cl, used = _table(rig, [L[i]["ring"], L[i]["arm"]], [L[j]["ring"], L[j]["arm"]],
                          lambda p: rot_about_z(math.radians(p)), phis)
        lo, hi = _free(phis, cl)
        d0 = math.degrees(rig.th0[j] - rig.th0[i])
        wrap = lambda x: (x + 180) % 360 - 180
        sep = min(abs(wrap(d0 + lo)), abs(wrap(d0 + hi)))     # angular distance at first contact
        out["ring_contact_sep_deg"].append(sep)
        out["pairs"][f"legs {i + 1}-{j + 1}"] = used
        log(f"  legs {i + 1}-{j + 1}: arms touch when the rings come within {sep:.1f} deg  {used}")
    # (b) elbow and wrist
    phis = np.arange(-180, 180.1, 1.0)
    for i in range(3):
        cl, used = _table(rig, [L[i]["arm"], L[i]["ring"]], [L[i]["mid"]],
                          lambda p: axis_angle(rig.W0[i], math.radians(p)), phis)
        out["elbow_deg"].append(list(_free(phis, cl)))
        out["pairs"][f"elbow {i + 1}"] = used
        cl, used = _table(rig, [L[i]["mid"]], rig.lay["femur"],
                          lambda p: axis_angle(rig.V0[i], math.radians(p)), phis)
        out["wrist_deg"].append(list(_free(phis, cl)))
        out["pairs"][f"wrist {i + 1}"] = used
        log(f"  leg {i + 1}: elbow free {out['elbow_deg'][-1]}, wrist free {out['wrist_deg'][-1]} deg (from the CAD pose)")
    return out


def general_pairs(rig: Rig):
    """Pairs NOT covered by the ring-separation / elbow / wrist tables."""
    L = rig.lay["legs"]
    covered = set()
    for i in range(3):
        mine = (L[i]["ring"], L[i]["arm"])
        for a in mine:
            covered |= {(a, L[i]["mid"]), (L[i]["mid"], a)}
            for f in rig.lay["fixed"]:
                covered |= {(a, f), (f, a)}                # z-rotation pairs: no collision (checked in 'baseline')
            for j in range(3):
                if j != i:
                    covered |= {(a, b) for b in (L[j]["ring"], L[j]["arm"])}
        for f in rig.lay["femur"]:
            covered |= {(L[i]["mid"], f), (f, L[i]["mid"])}
    return [p for p in rig.pairs if p not in covered]


def sweep_general(rig: Rig, limits_cfg: dict, log=print):
    """Grid over hip angles; for poses the controller's safety gate (with the
    new limits) ALLOWS, measure the general pairs.  Returns (min_mm, worst)."""
    from spm.controller import Controller
    ctl = Controller(rig.config_path)            # the config this rig was built from
    ctl.shutdown()
    ctl.cfg["limits"].update(limits_cfg)
    ctl.cfg["limits"]["max_tilt_deg"] = 180          # sweep the whole collision-free set, not the tilt cap
    gp = general_pairs(rig)
    best, worst, n = 99.0, None, 0
    for f in np.arange(-40, 121, 5.0):
        for a in np.arange(-45, 46, 5.0):
            for r in np.arange(-60, 61, 10.0):
                R = rig.hip.effector_from_anat(rig.hip.anat_matrix(f, a, r))
                th, m = rig.spm.ik(R, rig.th0)
                ok, why, met = ctl.check_pose(R, th, m)
                if not ok:
                    continue
                n += 1
                cl = rig.clearances(R, gp)
                k = min(cl, key=cl.get)
                if cl[k] < best:
                    best, worst = cl[k], (float(f), float(a), float(r), rig.label(k[0]), rig.label(k[1]))
    log(f"  {n} allowed poses swept, {len(gp)} general pairs: closest {best:.2f} mm at flex/add/rot "
        f"{worst[:3]} ({worst[3]} <-> {worst[4]})")
    return best, worst


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("step")
    ap.add_argument("what", choices=["baseline", "limits"])
    ap.add_argument("--ring-margin", type=float, default=1.5, help="deg added to the measured ring contact separation")
    ap.add_argument("--joint-margin", type=float, default=3.0, help="deg kept away from elbow/wrist contact")
    ap.add_argument("--config", default=str(ROOT / "config.json"), help="config describing THIS step file")
    ap.add_argument("--write", action="store_true",
                    help="write the measured limits into --config (refused for config.json itself)")
    args = ap.parse_args()
    rig = Rig(Path(args.step), Path(args.config))
    if args.what == "baseline":
        cl = rig.clearances(rig.R0)
        print("clearances under 15 mm at the CAD pose (designed contacts are expected here):")
        for (a, b), d in sorted(cl.items(), key=lambda x: x[1]):
            if d < rig.REACH_MM:
                print(f"  {d:6.2f} mm  {rig.label(a):30s} <-> {rig.label(b)}")
        return
    t0 = time.time()
    lim = compute_limits(rig)
    sep = max(lim["ring_contact_sep_deg"])
    cfg = {"min_ring_separation_deg": round(sep + args.ring_margin, 1),
           "joint_limits": {"_source": f"experiments/cad_collision.py on {Path(args.step).name}: contact (< {TOUCH_MM} mm) "
                                       f"angles relative to the CAD assembly pose (ref_R_BE); +-180 = never collides",
                            "ref_R_BE": np.round(rig.R0, 9).tolist(),
                            "elbow_deg": lim["elbow_deg"], "wrist_deg": lim["wrist_deg"],
                            "margin_deg": args.joint_margin}}
    best, worst = sweep_general(rig, cfg)
    status = "OK" if best >= GENERAL_MIN_MM else "NOT ENOUGH - a pose-dependent collision exists, needs a runtime check"
    print(f"\npose-dependent pairs: closest {best:.2f} mm -> {status}")
    print(f"ring contact separation {sep:.1f} deg; took {time.time() - t0:.0f} s\n")
    print("config.json 'limits' entries:")
    print(json.dumps(cfg, indent=2))
    if args.write:
        cp = Path(args.config)
        if cp.resolve() == (ROOT / "config.json").resolve():
            print("NOT written: refusing to modify config.json (copy the entries by hand if intended)")
        else:
            full = json.loads(cp.read_text())
            full["limits"].update(cfg)
            cp.write_text(json.dumps(full, indent=2) + "\n")
            print(f"limits written into {cp}")
    (HERE / "cad_cache" / f"limits_result_{Path(args.step).stem.replace(' ', '_')}.json").write_text(json.dumps(
        {"config": cfg, "detail": lim, "general_min_mm": best, "general_worst": worst}, indent=1))


if __name__ == "__main__":
    main()

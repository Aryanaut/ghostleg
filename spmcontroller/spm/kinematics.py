"""
kinematics.py - Kinematics of a 3-RRR spherical parallel manipulator whose three
input axes are COAXIAL (all three internal rings turn about the same base axis).

All angles in this file are RADIANS unless a name ends in _deg.

Frames
------
Base frame B (fixed to the gearbox housing):
    origin = centre of rotation (centre of the femoral head),
    +z     = the common ring axis, pointing from the base TOWARD the head,
    +x     = the direction a ring angle of theta = 0 points to.
Effector frame E (fixed to the femoral head / moving platform).
    The orientation of the effector is the rotation matrix R = R_BE.

Geometry of leg i (all joint axes pass through the origin, so only DIRECTIONS
matter - that is what makes it "spherical"):
    u      = -z                actuator (ring) axis
    w_i    middle joint axis   at angle alpha1_i from u, swung round z by theta_i:
                w_i(theta) = [ sin(a1) cos(theta), sin(a1) sin(theta), -cos(a1) ]
    v_i    effector joint axis, fixed in E at polar angle beta_i, azimuth eta_i:
                v_i = R @ [ sin(b) cos(eta), sin(b) sin(eta), cos(b) ]
    distal link constraint:   w_i . v_i = cos(alpha2_i)

Inverse kinematics (closed form, independent per leg):
    sin(a1)(vx cos(th) + vy sin(th)) - cos(a1) vz = cos(a2)
    =>  A cos(th) + B sin(th) = C,   A = sin(a1) vx, B = sin(a1) vy,
                                     C = cos(a2) + cos(a1) vz
    =>  th = atan2(B, A) + branch * acos(C / hypot(A, B))
    There are two solutions per leg ("branch" = +1/-1, the elbow direction);
    the configured branch is held fixed so the mechanism never flips.

Velocity relation used for singularity checks (differentiate the constraint):
    (z x w_i) . v_i * dtheta_i  =  (w_i x v_i) . omega
    =>  Jb * dtheta = Ja * omega,   Ja rows = (w_i x v_i),  Jb = diag((z x w_i).v_i)
    Jb singular  -> "serial"  singularity (a leg is fully stretched/folded)
    Ja singular  -> "parallel" singularity (the effector gains an uncontrolled DOF)
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

Z = np.array([0.0, 0.0, 1.0])


# ===========================================================================
# Small rotation helpers
# ===========================================================================

def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_angle(axis, angle: float) -> np.ndarray:
    """Rodrigues' formula: rotation of `angle` about unit-ised `axis`."""
    k = np.asarray(axis, float)
    n = np.linalg.norm(k)
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = k / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def rotvec(R: np.ndarray) -> np.ndarray:
    """Inverse of Rodrigues: rotation matrix -> rotation vector (axis * angle)."""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    if np.pi - ang < 1e-6:
        # Near 180 deg the antisymmetric part vanishes; use the symmetric part.
        M = (R + np.eye(3)) / 2
        k = np.sqrt(np.clip(np.diag(M), 0, None))
        i = int(np.argmax(k))
        k[:] = M[i] / k[i]
        return k / np.linalg.norm(k) * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2 * np.sin(ang)) * ang


def wrap_near(angle: float, ref: float) -> float:
    """Add/subtract 2*pi so `angle` is the representative closest to `ref`.
    Keeps ring angles continuous instead of jumping at +-180 deg."""
    return ref + (angle - ref + np.pi) % (2 * np.pi) - np.pi


# ===========================================================================
# Geometry container
# ===========================================================================

@dataclass
class SPMGeometry:
    """Per-leg geometric parameters (degrees, as the user types them)."""
    alpha1_deg: list = field(default_factory=lambda: [50.0, 50.0, 50.0])
    alpha2_deg: list = field(default_factory=lambda: [90.0, 90.0, 90.0])
    beta_deg: list = field(default_factory=lambda: [90.0, 90.0, 90.0])
    eta_deg: list = field(default_factory=lambda: [0.0, 120.0, 240.0])
    branch: list = field(default_factory=lambda: [1, 1, 1])

    @classmethod
    def from_dict(cls, d: dict) -> "SPMGeometry":
        g = cls()
        for k in ("alpha1_deg", "alpha2_deg", "beta_deg", "eta_deg", "branch"):
            if k in d:
                val = d[k]
                # Allow a single number to mean "same for all three legs".
                setattr(g, k, list(val) if isinstance(val, (list, tuple)) else [val] * 3)
        return g

    # Pre-computed radians as numpy arrays (cheap; recomputed on access).
    @property
    def a1(self): return np.radians(self.alpha1_deg)
    @property
    def a2(self): return np.radians(self.alpha2_deg)
    @property
    def b(self): return np.radians(self.beta_deg)
    @property
    def eta(self): return np.radians(self.eta_deg)


# ===========================================================================
# Core kinematics
# ===========================================================================

class CoaxialSPM:
    def __init__(self, geom: SPMGeometry):
        self.geom = geom

    # ---- joint axis directions ---------------------------------------------

    def v_local(self) -> np.ndarray:
        """3x3, row i = effector joint axis v_i expressed in the EFFECTOR frame."""
        g = self.geom
        return np.stack([np.sin(g.b) * np.cos(g.eta),
                         np.sin(g.b) * np.sin(g.eta),
                         np.cos(g.b)], axis=1)

    def v_world(self, R: np.ndarray) -> np.ndarray:
        """Rows = v_i in the BASE frame for effector orientation R."""
        return (R @ self.v_local().T).T

    def w_world(self, theta: np.ndarray) -> np.ndarray:
        """Rows = middle joint axes w_i for ring angles theta (radians)."""
        a1 = self.geom.a1
        return np.stack([np.sin(a1) * np.cos(theta),
                         np.sin(a1) * np.sin(theta),
                         -np.cos(a1) * np.ones_like(theta)], axis=1)

    # ---- inverse kinematics ------------------------------------------------

    def ik(self, R: np.ndarray, theta_ref: np.ndarray | None = None):
        """Ring angles for effector orientation R.

        Returns (theta[3], margin[3]).  margin_i = 1 - |C|/hypot(A,B) is how far
        leg i is from its reach limit (0 = exactly at a serial singularity,
        negative = pose unreachable; theta is then NaN for that leg).
        theta_ref (optional) is used to unwrap each angle next to the previous
        value so commands stay continuous.
        """
        g = self.geom
        V = self.v_world(R)
        A = np.sin(g.a1) * V[:, 0]
        B = np.sin(g.a1) * V[:, 1]
        C = np.cos(g.a2) + np.cos(g.a1) * V[:, 2]
        rho = np.hypot(A, B)
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(rho > 1e-12, C / rho, np.inf)
        margin = 1.0 - np.abs(ratio)
        theta = np.arctan2(B, A) + np.asarray(g.branch) * np.arccos(np.clip(ratio, -1, 1))
        theta = np.where(margin >= 0, theta, np.nan)
        if theta_ref is not None:
            theta = np.array([wrap_near(t, r) if np.isfinite(t) else t
                              for t, r in zip(theta, theta_ref)])
        return theta, margin

    # ---- forward kinematics --------------------------------------------------

    def fk(self, theta: np.ndarray, R_guess: np.ndarray, iters: int = 30, tol: float = 1e-10):
        """Effector orientation for ring angles theta, by Newton iteration.

        Residual f_i(R) = w_i . v_i(R) - cos(alpha2_i).  A small rotation delta
        applied as R <- exp(delta) R changes v_i by delta x v_i, hence
        df_i = (v_i x w_i) . delta.  Solve the 3x3 system each step.

        A parallel mechanism has several FK solutions; starting from the last
        known pose (R_guess) picks the physically continuous one.
        Returns (R, converged: bool).
        """
        W = self.w_world(np.asarray(theta, float))
        c2 = np.cos(self.geom.a2)
        R = R_guess.copy()
        for _ in range(iters):
            V = self.v_world(R)
            f = np.einsum("ij,ij->i", W, V) - c2
            if np.max(np.abs(f)) < tol:
                return R, True
            J = np.cross(V, W)
            try:
                delta = -np.linalg.solve(J, f)
            except np.linalg.LinAlgError:
                return R, False
            # Limit the step so a bad guess can't jump to another assembly mode.
            n = np.linalg.norm(delta)
            if n > 0.2:
                delta *= 0.2 / n
            R = axis_angle(delta, np.linalg.norm(delta)) @ R
        V = self.v_world(R)
        ok = np.max(np.abs(np.einsum("ij,ij->i", W, V) - c2)) < 1e-6
        return R, bool(ok)

    # ---- Jacobians / singularity measures -----------------------------------

    def singularity_metrics(self, R: np.ndarray, theta: np.ndarray) -> dict:
        """Dimensionless 0..1 numbers; small = close to a singularity.

        parallel_index : |det| of the matrix whose rows are the UNIT vectors
                         (w_i x v_i).  1 = the three are mutually orthogonal,
                         0 = coplanar (parallel singularity).
        serial_index   : min_i |(z x w_i).v_i| / |z x w_i|, i.e. how much
                         turning ring i actually moves its constraint.
        cond           : condition number of J = Jb^-1 Ja (dtheta = J omega).
        """
        W = self.w_world(theta)
        V = self.v_world(R)
        Ja = np.cross(W, V)
        rows = Ja / np.maximum(np.linalg.norm(Ja, axis=1, keepdims=True), 1e-12)
        parallel = abs(float(np.linalg.det(rows)))
        zw = np.cross(Z, W)
        jb = np.einsum("ij,ij->i", zw, V)
        serial = float(np.min(np.abs(jb) / np.maximum(np.linalg.norm(zw, axis=1), 1e-12)))
        try:
            J = Ja / jb[:, None]
            cond = float(np.linalg.cond(J))
        except Exception:
            cond = float("inf")
        return {"parallel_index": parallel, "serial_index": serial, "cond": cond}

    # ---- passive joint angles (for link-collision limits) --------------------

    def joint_fold_angles(self, R: np.ndarray, theta: np.ndarray,
                          R_ref: np.ndarray, theta_ref: np.ndarray):
        """How far each leg's two PASSIVE joints have turned since a reference
        pose (degrees, signed, in (-180, 180]).

        elbow_i : the middle (distal) link relative to the proximal arm, about
                  the middle joint axis w_i.  The arm turns rigidly with its
                  ring, so the arm carries the reference v_i to
                  Rz(theta_i - theta_ref_i) @ v_ref_i; the elbow angle is the
                  rotation about w_i from there to the actual v_i.
        wrist_i : the distal link relative to the femur head, about the head
                  joint axis v_i.  The head carries the reference w_i to
                  (R @ R_ref^T) @ w_ref_i; the wrist angle is the rotation
                  about v_i from there to the actual w_i.

        Both are 0 at the reference pose.  Link collisions measured in the CAD
        (experiments/cad_collision.py) become simple intervals on these angles,
        because each joint is a single rotation between two rigid bodies."""
        W, V = self.w_world(theta), self.v_world(R)
        W0, V0 = self.w_world(theta_ref), self.v_world(R_ref)
        elbow, wrist = [], []
        for i in range(3):
            dz = theta[i] - theta_ref[i]
            c, s = np.cos(dz), np.sin(dz)
            v_ref = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ V0[i]
            elbow.append(float(np.degrees(np.arctan2(np.cross(v_ref, V[i]) @ W[i], v_ref @ V[i]))))
            w_ref = R @ R_ref.T @ W0[i]
            wrist.append(float(np.degrees(np.arctan2(np.cross(w_ref, W[i]) @ V[i], w_ref @ W[i]))))
        return elbow, wrist

    # ---- geometry for drawing ---------------------------------------------

    @staticmethod
    def _arc(p: np.ndarray, q: np.ndarray, radius: float, n: int = 16) -> list:
        """Points on the great-circle arc from direction p to direction q."""
        p = p / np.linalg.norm(p)
        q = q / np.linalg.norm(q)
        ang = np.arccos(np.clip(p @ q, -1, 1))
        if ang < 1e-6:
            return [list(p * radius)] * 2
        axis = np.cross(p, q)
        if np.linalg.norm(axis) < 1e-9:           # antiparallel: choose any normal
            axis = np.cross(p, [1, 0, 0]) if abs(p[0]) < 0.9 else np.cross(p, [0, 1, 0])
        return [list(axis_angle(axis, ang * t) @ p * radius) for t in np.linspace(0, 1, n)]

    def drawing(self, R: np.ndarray, theta: np.ndarray, r_prox: float, r_dist: float) -> dict:
        """Polylines (in the BASE frame, metres) describing the linkage, for the 3D view.
        Proximal links are drawn as arcs from the ring (just above the -z pole)
        to w_i, distal links as arcs from w_i to v_i."""
        W = self.w_world(theta)
        V = self.v_world(R)
        legs = []
        for i in range(3):
            # Ring "attachment" direction: 15 deg off the -z axis at the ring's azimuth,
            # purely cosmetic so the three proximal links are visually distinct.
            start = np.array([np.sin(np.radians(15)) * np.cos(theta[i]),
                              np.sin(np.radians(15)) * np.sin(theta[i]),
                              -np.cos(np.radians(15))])
            legs.append({
                "proximal": self._arc(start, W[i], r_prox),
                "distal": self._arc(W[i], V[i], r_dist),
                "w": list(W[i]),
                "v": list(V[i]),
            })
        return {"legs": legs}

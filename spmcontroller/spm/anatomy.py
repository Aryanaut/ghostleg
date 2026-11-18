"""
anatomy.py - Map clinical hip angles onto the SPM's effector orientation.

Pelvis frame P follows the ISB recommendation (Wu et al., J Biomech 2002) for
the hip, written for the RIGHT leg:
    X = anterior,  Y = superior,  Z = to the right (lateral for a right hip)
    origin = hip joint centre = centre of the femoral head = SPM centre.

Hip joint angles use the ISB / Grood-Suntay joint coordinate system, which is
equivalent to the intrinsic Z-X'-Y'' Euler sequence:
    R_anat = Rz(flexion) @ Rx(adduction) @ Ry(internal rotation)
Sign checks for the right leg (femur hangs along -Y at neutral):
    Rz(+f): -Y -> +X  : knee moves anterior   -> flexion   positive
    Rx(+a): -Y -> -Z  : knee moves medial     -> adduction positive
    Ry(+r): +X -> -Z  : femur front turns medial -> internal rotation positive
For a LEFT leg, medial is +Z, so adduction and rotation change sign
(the ISB "mirror z" rule); flexion is unchanged.

How the SPM sits on the pelvis (the "mount")
--------------------------------------------
The SPM base axis z_B (ring axis, base -> head) is described in the pelvis
frame by two angles that mirror how an acetabulum is described clinically:
    tilt_down_deg  : how far z_B is tipped below the horizontal lateral direction
    forward_deg    : how far z_B is swung toward anterior
plus base_twist_deg, a spin of the base about its own axis (which ring
angle "0" points where).  From the second CAD picture the default guess is the
ring axis pointing lateral-inferior-anterior, like the acetabular opening.

Effector orientation that realises a hip pose:
    R_BE = R_PB^T @ R_anat @ R_PB @ R_neutral
where R_PB is the base orientation in the pelvis frame and R_neutral is the
effector orientation (in the base frame) when the leg is in anatomical
neutral (normally the mechanism's symmetric "home": identity).
"""

from __future__ import annotations

import numpy as np

from .kinematics import axis_angle, rot_x, rot_y, rot_z


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


class HipMapping:
    def __init__(self, cfg: dict):
        """cfg = the "mount" + "femur" sections of config.json."""
        self.update(cfg)

    def update(self, cfg: dict) -> None:
        m = cfg.get("mount", {})
        self.side = 1 if m.get("side", "right") == "right" else -1
        tilt = np.radians(m.get("tilt_down_deg", 45.0))
        fwd = np.radians(m.get("forward_deg", 20.0))
        twist = np.radians(m.get("base_twist_deg", 0.0))

        # --- base axis z_B in the pelvis frame -----------------------------
        lateral = np.array([0, 0, self.side], float)
        z_b = unit(np.cos(tilt) * np.cos(fwd) * lateral      # mostly lateral
                   + np.cos(tilt) * np.sin(fwd) * np.array([1, 0, 0])   # a bit anterior
                   - np.sin(tilt) * np.array([0, 1, 0]))     # tipped down
        # --- base x axis: anterior, made perpendicular to z_B, then twisted --
        x_b = unit(np.array([1, 0, 0]) - (np.array([1, 0, 0]) @ z_b) * z_b)
        x_b = axis_angle(z_b, twist) @ x_b
        y_b = np.cross(z_b, x_b)
        self.R_PB = np.column_stack([x_b, y_b, z_b])         # base axes in pelvis frame

        # --- neutral effector pose (small corrections in the base frame) ----
        n = cfg.get("neutral", {})
        self.R_neutral = (rot_z(np.radians(n.get("twist_deg", 0.0)))
                          @ rot_y(np.radians(n.get("tilt_y_deg", 0.0)))
                          @ rot_x(np.radians(n.get("tilt_x_deg", 0.0))))

        # --- femur stick model for the 3D view (metres, pelvis frame) ------
        f = cfg.get("femur", {})
        neck = f.get("neck_length_m", 0.05)
        length = f.get("mech_length_m", 0.42)
        head_r = f.get("head_radius_m", 0.024)
        # The femoral neck runs along the femur-head part's symmetry axis,
        # which is the EFFECTOR z axis (measured from the CAD: the head's
        # screw holes, which hold it on the femur, lie on that axis). At
        # neutral, that axis sits at R_neutral @ z in the base frame,
        # i.e. R_PB @ R_neutral @ z in the pelvis frame.  (With the old
        # default R_neutral = I this reduces to the ring axis, as before.)
        neck_end = self.R_PB @ self.R_neutral[:, 2] * neck
        knee = np.array([0, -length, 0])                        # mechanical axis ~ vertical
        # medial/lateral epicondyles, to visualise axial rotation
        epi = np.array([0, 0, self.side]) * 0.04
        self.femur0 = {"head": np.zeros(3), "head_r": head_r, "neck_end": neck_end,
                       "knee": knee, "epi_lat": knee + epi, "epi_med": knee - epi,
                       "toe": knee + np.array([0.06, 0, 0])}     # anterior marker

    # ------------------------------------------------------------------
    def anat_matrix(self, flex_deg: float, add_deg: float, rot_deg: float) -> np.ndarray:
        """R_anat in the pelvis frame for given clinical angles (degrees)."""
        s = self.side
        return (rot_z(np.radians(flex_deg))
                @ rot_x(np.radians(s * add_deg))
                @ rot_y(np.radians(s * rot_deg)))

    def anat_angles(self, R_anat: np.ndarray) -> tuple[float, float, float]:
        """Inverse of anat_matrix: Z-X-Y Euler decomposition -> (flex, add, rot) deg.
        For R = Rz(f) Rx(a) Ry(r):  R[2,1] = sin(a),
        R[0,1] = -cos(a) sin(f), R[1,1] = cos(a) cos(f),
        R[2,0] = -cos(a) sin(r), R[2,2] = cos(a) cos(r)."""
        a = np.arcsin(np.clip(R_anat[2, 1], -1, 1))
        f = np.arctan2(-R_anat[0, 1], R_anat[1, 1])
        r = np.arctan2(-R_anat[2, 0], R_anat[2, 2])
        s = self.side
        return float(np.degrees(f)), float(np.degrees(a)) * s, float(np.degrees(r)) * s

    def effector_from_anat(self, R_anat: np.ndarray) -> np.ndarray:
        """Hip rotation (pelvis frame) -> effector orientation R_BE (base frame)."""
        return self.R_PB.T @ R_anat @ self.R_PB @ self.R_neutral

    def anat_from_effector(self, R_BE: np.ndarray) -> np.ndarray:
        return self.R_PB @ R_BE @ self.R_neutral.T @ self.R_PB.T

    def femur_points(self, R_anat: np.ndarray) -> dict:
        """Femur landmarks rotated by the hip pose (pelvis frame), for drawing."""
        out = {}
        for k, p in self.femur0.items():
            out[k] = (R_anat @ p).tolist() if isinstance(p, np.ndarray) else p
        return out

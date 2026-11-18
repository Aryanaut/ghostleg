"""
motion.py - Motion "modes": each one answers "where should the hip be at time t?"

Every mode exposes
    target(t) -> ("anat", R_anat)   a hip rotation in the pelvis frame, or
                 ("joint", theta_offsets_rad)  ring angles relative to home
    timed     -> True if the mode moves by itself as time advances.

Time t is NOT wall-clock time.  The controller only advances t while the
mechanism is actually keeping up with the target (see controller.py), so a
gait cycle never "starts without you" while the leg is still travelling
from wherever it was to the first gait pose.
"""

from __future__ import annotations

import csv
import io
import math

import numpy as np

from .kinematics import axis_angle


# ===========================================================================
# Periodic interpolation (Catmull-Rom) - avoids needing scipy on Windows
# ===========================================================================

class PeriodicCurve:
    """Smooth closed curve through (phase%, value) key points, phase in [0,100)."""

    def __init__(self, keys: list[tuple[float, float]]):
        keys = sorted((float(p) % 100.0, float(v)) for p, v in keys)
        # Drop a duplicate 100% key that equals 0% after the modulo.
        dedup = []
        for p, v in keys:
            if dedup and abs(dedup[-1][0] - p) < 1e-9:
                continue
            dedup.append((p, v))
        if len(dedup) < 3:
            raise ValueError("need at least 3 distinct key points")
        self.p = np.array([k[0] for k in dedup])
        self.v = np.array([k[1] for k in dedup])

    def __call__(self, phase: float) -> float:
        p, v, n = self.p, self.v, len(self.p)
        x = phase % 100.0
        # Index of the key at or before x (wrapping)
        i1 = int(np.searchsorted(p, x, side="right") - 1) % n
        i2 = (i1 + 1) % n
        i0 = (i1 - 1) % n
        i3 = (i2 + 1) % n
        # Unwrapped phase positions of the four control points
        x1 = p[i1] if p[i1] <= x else p[i1] - 100
        x2 = p[i2] if p[i2] > x1 else p[i2] + 100
        x0 = p[i0] if p[i0] < x1 else p[i0] - 100
        x3 = p[i3] if p[i3] > x2 else p[i3] + 100
        # Non-uniform Catmull-Rom tangents (finite differences)
        m1 = (v[i2] - v[i0]) / (x2 - x0)
        m2 = (v[i3] - v[i1]) / (x3 - x1)
        h = x2 - x1
        s = (x - x1) / h
        # Cubic Hermite basis
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1
        h10 = s ** 3 - 2 * s ** 2 + s
        h01 = -2 * s ** 3 + 3 * s ** 2
        h11 = s ** 3 - s ** 2
        return float(h00 * v[i1] + h10 * h * m1 + h01 * v[i2] + h11 * h * m2)


# ---------------------------------------------------------------------------
# Built-in approximate normal-walking hip profile (degrees, % of gait cycle,
# 0% = initial contact of this leg).  Key values are textbook ranges:
#   flexion  ~30 deg at initial contact and loading response, max extension
#            ~10 deg just before toe-off (~50-60%), ~30 deg flexion in late swing
#   frontal  ~neutral at contact, ~10 deg adduction in loading response,
#            ~5 deg abduction in swing
#   transverse: small (several degrees); this shape is the LEAST certain.
# These are an illustrative shape for testing, not measured data. Load a
# CSV from your own gait lab / a public dataset for anything quantitative.
# ---------------------------------------------------------------------------
DEFAULT_GAIT_KEYS = {
    "flex": [(0, 30), (10, 29), (30, 12), (50, -10), (60, -2), (73, 18), (87, 31)],
    "add":  [(0, 0), (12, 9), (30, 6), (50, 2), (62, -4), (75, -5), (90, -2)],
    "rot":  [(0, 2), (15, 5), (50, -2), (65, -4), (85, 0)],
}


# ===========================================================================
# Modes
# ===========================================================================

class PoseMode:
    """Hold a fixed hip pose set by three sliders (flexion, adduction, rotation)."""
    name = "pose"
    timed = False

    def __init__(self, hip, flex=0.0, add=0.0, rot=0.0):
        self.hip, self.flex, self.add, self.rot = hip, flex, add, rot

    def target(self, t):
        return "anat", self.hip.anat_matrix(self.flex, self.add, self.rot)

    def describe(self):
        return {"flex": self.flex, "add": self.add, "rot": self.rot}


class PlaneMode:
    """Rotate the femur inside an arbitrary plane through the hip centre.

    The plane is given by its NORMAL n (= the rotation axis), specified in the
    pelvis frame by
        azimuth_deg   : 0 = medio-lateral axis (sagittal plane swing, i.e. flex/ext)
                       90 = antero-posterior axis (frontal plane, ab/adduction)
                       anything in between = an oblique vertical plane
        elevation_deg : tilts the axis up toward superior; 90 = vertical axis
                       (transverse plane, i.e. axial rotation of the femur).
    angle(t) = offset + amplitude * sin(2 pi f t)   (amplitude 0 -> static angle)
    The plane rotation is applied ON TOP of a base pose (flex/add/rot),
    so you can, e.g., swing obliquely around a slightly flexed posture.
    """
    name = "plane"
    timed = True

    def __init__(self, hip, azimuth_deg=0.0, elevation_deg=0.0, offset_deg=0.0,
                 amplitude_deg=0.0, freq_hz=0.5, base=(0.0, 0.0, 0.0)):
        self.hip = hip
        self.azimuth_deg, self.elevation_deg = azimuth_deg, elevation_deg
        self.offset_deg, self.amplitude_deg, self.freq_hz = offset_deg, amplitude_deg, freq_hz
        self.base = tuple(base)

    def axis(self) -> np.ndarray:
        az, el = math.radians(self.azimuth_deg), math.radians(self.elevation_deg)
        lateral = np.array([0, 0, self.hip.side], float)
        anterior = np.array([1, 0, 0], float)
        horiz = math.cos(az) * lateral + math.sin(az) * anterior
        return math.cos(el) * horiz + math.sin(el) * np.array([0, 1, 0])

    def angle(self, t) -> float:
        return self.offset_deg + self.amplitude_deg * math.sin(2 * math.pi * self.freq_hz * t)

    def target(self, t):
        R_base = self.hip.anat_matrix(*self.base)
        return "anat", axis_angle(self.axis(), math.radians(self.angle(t))) @ R_base

    def describe(self):
        return {"azimuth_deg": self.azimuth_deg, "elevation_deg": self.elevation_deg,
                "offset_deg": self.offset_deg, "amplitude_deg": self.amplitude_deg,
                "freq_hz": self.freq_hz, "base": list(self.base),
                "axis": self.axis().tolist()}


class GaitMode:
    """Replay a hip gait cycle (flexion, adduction, rotation vs % cycle)."""
    name = "gait"
    timed = True

    def __init__(self, hip, period_s=2.0, scale=(1.0, 1.0, 1.0), keys=None, label="built-in"):
        self.hip = hip
        self.period_s = period_s
        self.scale = tuple(scale)
        self.set_keys(keys or DEFAULT_GAIT_KEYS, label)

    def set_keys(self, keys: dict, label: str):
        self.curves = {k: PeriodicCurve(keys[k]) for k in ("flex", "add", "rot")}
        self.label = label

    def phase(self, t) -> float:
        return (t / self.period_s * 100.0) % 100.0

    def angles(self, phase):
        return tuple(self.curves[k](phase) * s for k, s in zip(("flex", "add", "rot"), self.scale))

    def target(self, t):
        return "anat", self.hip.anat_matrix(*self.angles(self.phase(t)))

    def preview(self, n=101):
        """Sampled curves for the GUI plot."""
        ph = np.linspace(0, 100, n)
        rows = [self.angles(p) for p in ph]
        return {"phase": ph.tolist(), "flex": [r[0] for r in rows],
                "add": [r[1] for r in rows], "rot": [r[2] for r in rows]}

    def describe(self):
        return {"period_s": self.period_s, "scale": list(self.scale), "label": self.label}


class EffectorMode:
    """Hold an effector orientation given directly in the SPM base frame.
    Used by automated routines (IMU mount calibration) that need rotations
    about the effector's own axes rather than clinical hip angles."""
    name = "effector"
    timed = False

    def __init__(self, R_BE=None):
        self.R_BE = np.eye(3) if R_BE is None else np.asarray(R_BE, float)

    def target(self, t):
        return "effector", self.R_BE

    def describe(self):
        return {"R_BE": self.R_BE.tolist()}


class JointMode:
    """Direct ring-angle jogging, relative to home (for commissioning: checking
    motor directions / which motor is which leg)."""
    name = "joint"
    timed = False

    def __init__(self, offsets_deg=(0.0, 0.0, 0.0)):
        self.offsets_deg = list(offsets_deg)

    def target(self, t):
        return "joint", np.radians(self.offsets_deg)

    def describe(self):
        return {"offsets_deg": self.offsets_deg}


# ===========================================================================
# CSV gait import
# ===========================================================================

def parse_gait_csv(text: str) -> dict:
    """Read a CSV with a header row containing a phase column and three angle
    columns.  Accepted header names (case-insensitive):
        phase / pct / percent / cycle      (0-100)   or   time (any unit; rescaled to 0-100)
        flex / flexion,  add / adduction,  rot / rotation / internal
    Missing angle columns are treated as zero."""
    reader = csv.DictReader(io.StringIO(text.strip()))
    if not reader.fieldnames:
        raise ValueError("CSV has no header row")
    names = {n.strip().lower(): n for n in reader.fieldnames}

    def find(*cands):
        for c in cands:
            for low, orig in names.items():
                if low.startswith(c):
                    return orig
        return None

    col_p = find("phase", "pct", "percent", "cycle", "time", "t")
    col_f = find("flex")
    col_a = find("add", "abd")
    col_r = find("rot", "int")
    if col_p is None:
        raise ValueError("CSV needs a phase/percent or time column")
    rows = [r for r in reader if r.get(col_p, "").strip()]
    if len(rows) < 3:
        raise ValueError("CSV needs at least 3 rows")
    ph = np.array([float(r[col_p]) for r in rows])
    ph = (ph - ph.min()) / (ph.max() - ph.min()) * 100.0     # normalise to 0..100
    keep = ph < 100.0 - 1e-9                                  # drop the closing sample
    out = {}
    for key, col in (("flex", col_f), ("add", col_a), ("rot", col_r)):
        vals = np.array([float(r[col]) if col else 0.0 for r in rows])
        if col == col_a and col and col.lower().startswith("abd"):
            vals = -vals                                     # abduction column -> adduction
        out[key] = list(zip(ph[keep].tolist(), vals[keep].tolist()))
    return out

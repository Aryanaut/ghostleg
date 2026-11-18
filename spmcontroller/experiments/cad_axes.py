"""
cad_axes.py - Pull the SPM's joint axes out of the Onshape STEP export, with
no CAD library needed.  (Analysis tool - kept in experiments/ on purpose.)

    python experiments/cad_axes.py "bone-in SPM.step"            # summary
    python experiments/cad_axes.py "bone-in SPM.step" --json out.json

What it does
------------
1. Reads the STEP (ISO 10303-21) text into {entity id: (type, args)}.
   Complex entities  #n=(A(..) B(..))  become type "COMPLEX" with a dict.
2. Works out where every part sits in the assembly: each
   NEXT_ASSEMBLY_USAGE_OCCURRENCE has a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
   whose REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION carries an
   ITEM_DEFINED_TRANSFORMATION(item1, item2).  The part's own coordinates map
   into the assembly by  T = A(item2) @ inv(A(item1))  (A = 4x4 from an
   AXIS2_PLACEMENT_3D).
3. For each part's solid(s), collects every CYLINDRICAL_SURFACE (bores, pins,
   bosses: these ARE the revolute joints) and SPHERICAL_SURFACE, transformed
   into assembly coordinates, and merges coaxial cylinders into one 'axis'.
4. Finds the centre of rotation (the femoral-head sphere centre, cross-checked
   against the point closest to all the joint axes) and lists, per part, the
   axes that pass through it (within --tol mm).  Those are the SPM joint axes.

Units: the file declares metres (SI_UNIT .METRE.); output is in mm.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# 1. STEP text -> entities
# ---------------------------------------------------------------------------

def split_records(text: str):
    """Yield 'id', 'body' for each  #id = body ;  in the DATA section,
    honouring quotes (a ';' inside a string does not end the record)."""
    data = text[text.index("DATA;") + 5:]
    i, n = 0, len(data)
    buf = []
    in_str = False
    start = 0
    while i < n:
        c = data[i]
        if c == "'":
            in_str = not in_str            # '' (escaped quote) toggles twice: harmless
        elif c == ";" and not in_str:
            rec = data[start:i].strip()
            start = i + 1
            if rec.startswith("#"):
                eq = rec.index("=")
                yield int(rec[1:eq]), rec[eq + 1:].strip()
            elif rec.startswith("ENDSEC"):
                return
        i += 1


_tok = re.compile(r"\s*(#\d+|'(?:[^']|'')*'|\.[A-Z_0-9]+\.|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?\.?|\$|\*|[A-Z_][A-Z_0-9]*|\(|\)|,)")


def parse_args(s: str):
    """Parse a STEP parameter list '( ... )' into nested Python lists.
    #12 -> ('ref', 12); numbers -> float; strings -> str; enums -> '.X.'."""
    toks = [m.group(1) for m in _tok.finditer(s)]
    pos = 0

    def val():
        nonlocal pos
        t = toks[pos]
        pos += 1
        if t == "(":
            out = []
            while toks[pos] != ")":
                out.append(val())
                if toks[pos] == ",":
                    pos += 1
            pos += 1
            return out
        if t.startswith("#"):
            return ("ref", int(t[1:]))
        if t.startswith("'"):
            return t[1:-1]
        if t[0] in "-+.0123456789" and not (t.startswith(".") and t.endswith(".") and len(t) > 1 and t[1].isalpha()):
            try:
                return float(t.rstrip(".")) if t not in ("-", "+") else t
            except ValueError:
                return t
        if re.match(r"[A-Z_]", t) and pos < len(toks) and toks[pos] == "(":
            return (t, val())                       # typed value, e.g. LENGTH_MEASURE(1.)
        return t

    return val()


def load(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    ents = {}
    for eid, body in split_records(text):
        if body.startswith("("):                    # complex entity: (A(...) B(...) ...)
            # parse_args turns 'NAME(args)' into (NAME, [args]) and accepts
            # items with no commas between them, which is exactly this form.
            # (Used for rational B-splines and the assembly relationships.)
            ents[eid] = ("COMPLEX", {name: args for name, args in parse_args(body)})
        else:
            k = body.index("(")
            ents[eid] = (body[:k].strip(), parse_args(body[k:]))
    return ents


# ---------------------------------------------------------------------------
# 2. geometry helpers
# ---------------------------------------------------------------------------

def R(ref):
    return ref[1]


def point(E, ref):
    t, a = E[R(ref)]
    assert t == "CARTESIAN_POINT", t
    return np.array(a[1], float)


def direction(E, ref):
    t, a = E[R(ref)]
    assert t == "DIRECTION", t
    v = np.array(a[1], float)
    return v / np.linalg.norm(v)


def placement(E, ref) -> np.ndarray:
    """AXIS2_PLACEMENT_3D -> 4x4 (columns: x, y, z axes, origin)."""
    t, a = E[R(ref)]
    assert t == "AXIS2_PLACEMENT_3D", t
    o = point(E, a[1])
    z = direction(E, a[2]) if a[2] != "$" else np.array([0, 0, 1.0])
    x = direction(E, a[3]) if a[3] != "$" else (np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0]))
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, o
    return M


def refs_in(obj):
    """All ('ref', n) inside a nested arg structure."""
    if isinstance(obj, tuple) and len(obj) == 2 and obj[0] == "ref":
        yield obj[1]
    elif isinstance(obj, (list, tuple)):
        for x in obj:
            yield from refs_in(x)


# ---------------------------------------------------------------------------
# 3. assembly structure
# ---------------------------------------------------------------------------

def assembly(E):
    """Return list of parts: {name, nauo, T (4x4 part->assembly), solids[ids]}."""
    by_type = defaultdict(list)
    for eid, (t, a) in E.items():
        by_type[t].append(eid)

    # product_definition -> product name
    pd_name = {}
    for pd in by_type["PRODUCT_DEFINITION"]:
        form = E[R(E[pd][1][2])]                       # PRODUCT_DEFINITION_FORMATION...
        prod = E[R(form[1][2])]
        pd_name[pd] = prod[1][0]

    # PRODUCT_DEFINITION_SHAPE: definition -> pds id
    pds_of = {R(E[p][1][2]): p for p in by_type["PRODUCT_DEFINITION_SHAPE"]}
    # SHAPE_DEFINITION_REPRESENTATION: pds -> shape rep
    rep_of_pds = {R(E[s][1][0]): R(E[s][1][1]) for s in by_type["SHAPE_DEFINITION_REPRESENTATION"]}
    # SHAPE_REPRESENTATION_RELATIONSHIP: rep <-> rep (shape rep <-> brep rep)
    srr = defaultdict(set)
    for s in by_type["SHAPE_REPRESENTATION_RELATIONSHIP"]:
        a, b = R(E[s][1][2]), R(E[s][1][3])
        srr[a].add(b); srr[b].add(a)

    def solids_of_rep(rep, seen=None):
        seen = seen or set()
        if rep in seen:
            return []
        seen.add(rep)
        out = []
        t, a = E[rep]
        items = a[1] if t != "COMPLEX" else []
        for it in refs_in(items):
            if E[it][0] in ("MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS"):
                out.append(it)
        for other in srr.get(rep, ()):
            out += solids_of_rep(other, seen)
        return out

    # CONTEXT_DEPENDENT_SHAPE_REPRESENTATION: (rep_relationship, pds_of_nauo)
    cdsr_of_nauo = {}
    for c in by_type["CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"]:
        relref, pdsref = E[c][1][0], E[c][1][1]
        nauo = R(E[R(pdsref)][1][2])
        cdsr_of_nauo[nauo] = R(relref)

    parts = []
    for n in by_type["NEXT_ASSEMBLY_USAGE_OCCURRENCE"]:
        a = E[n][1]
        child_pd = R(a[4])
        rel = E[cdsr_of_nauo[n]]
        assert rel[0] == "COMPLEX", rel[0]
        rr = rel[1]["REPRESENTATION_RELATIONSHIP"]
        rep1, rep2 = R(rr[2]), R(rr[3])
        idt = R(rel[1]["REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"][0])
        it1, it2 = E[idt][1][2], E[idt][1][3]
        T = placement(E, it2) @ np.linalg.inv(placement(E, it1))
        child_rep = rep_of_pds[pds_of[child_pd]]
        # rep1 should be the child's rep (STEP convention); warn if not
        if child_rep not in (rep1, rep2):
            print(f"  ! {pd_name[child_pd]}: child rep not in relationship", file=sys.stderr)
        parts.append({"name": pd_name[child_pd], "nauo": a[0], "T": T,
                      "solids": solids_of_rep(child_rep)})
    return parts


def surfaces_of_solid(E, sid):
    """ADVANCED_FACE surfaces reachable from a solid."""
    out = []
    shell = R(E[sid][1][1])
    for f in refs_in(E[shell][1][1]):
        t, a = E[f]
        if t == "ADVANCED_FACE":
            out.append(R(a[2]))
    return out


# ---------------------------------------------------------------------------
# 4. axes
# ---------------------------------------------------------------------------

def part_features(E, part):
    """Cylinders (axis lines) and spheres of one part, in assembly mm."""
    T = part["T"]
    cyl, sph = [], []
    for s in part["solids"]:
        for srf in surfaces_of_solid(E, s):
            t, a = E[srf]
            if t == "CYLINDRICAL_SURFACE":
                M = T @ placement(E, a[1])
                cyl.append((M[:3, 3] * 1000, M[:3, 2], a[2] * 1000))
            elif t == "SPHERICAL_SURFACE":
                M = T @ placement(E, a[1])
                sph.append((M[:3, 3] * 1000, a[2] * 1000))
    return cyl, sph


def merge_axes(cyl, ang_tol=0.5, dist_tol=0.3):
    """Group coaxial cylinders -> [{p, d, radii, n}] (p = a point on the line)."""
    groups = []
    for p, d, r in cyl:
        for g in groups:
            if math.degrees(math.acos(min(1, abs(float(d @ g["d"]))))) < ang_tol:
                off = p - g["p"]
                if np.linalg.norm(off - (off @ g["d"]) * g["d"]) < dist_tol:
                    g["radii"].add(round(r, 2)); g["n"] += 1
                    break
        else:
            groups.append({"p": p, "d": d * (1 if d[np.argmax(np.abs(d))] > 0 else -1),
                           "radii": {round(r, 2)}, "n": 1})
    return groups


def line_dist(c, g):
    off = c - g["p"]
    return float(np.linalg.norm(off - (off @ g["d"]) * g["d"]))


def closest_point_to_lines(lines):
    """Least-squares point nearest a set of lines (p, d)."""
    A = np.zeros((3, 3)); b = np.zeros(3)
    for p, d in lines:
        P = np.eye(3) - np.outer(d, d)
        A += P; b += P @ p
    return np.linalg.solve(A, b)


# ---------------------------------------------------------------------------
# 5. colours: part colours, faces coloured differently, the GROUND face
# ---------------------------------------------------------------------------
#
# STEP (AP214/AP242) attaches colours with
#   STYLED_ITEM(name, (PRESENTATION_STYLE_ASSIGNMENT...), item)
#   -> SURFACE_STYLE_USAGE -> SURFACE_SIDE_STYLE -> SURFACE_STYLE_FILL_AREA
#   -> FILL_AREA_STYLE -> FILL_AREA_STYLE_COLOUR -> COLOUR_RGB(name, r, g, b)
# where `item` is a whole solid (MANIFOLD_SOLID_BREP) or a single face
# (ADVANCED_FACE).  A face styled differently from its solid is a "marked"
# face.  Convention for this rig (user, 2026-11-18): a marked PLANAR face is
# the one that is to be PARALLEL TO THE GROUND.

def _colour(E, ref):
    """Follow any references from a style down to its colour (r, g, b) 0..1."""
    stack, seen = [ref], set()
    while stack:
        e = stack.pop()
        if e in seen:
            continue
        seen.add(e)
        t, a = E[e]
        if t == "COLOUR_RGB":
            return tuple(round(float(x), 4) for x in a[1:4])
        if t == "DRAUGHTING_PRE_DEFINED_COLOUR":
            return a[0]                                   # a named colour, e.g. 'red'
        stack.extend(refs_in(a))
    return None


def styled_colours(E):
    """{styled entity id: colour} for every STYLED_ITEM (solids and faces).
    Onshape writes a face coloured differently from its part as an
    OVER_RIDING_STYLED_ITEM(name, styles, item, over_ridden_style): same
    first three arguments, plus the part-level style it overrides."""
    out = {}
    for eid, (t, a) in E.items():
        if t in ("STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"):
            # only the item's OWN style chain (not the overridden style, which is the
            # 4th argument) - so walk from the styles argument, not the whole entity
            out[R(a[2])] = _colour(E, R(a[1][0])) if a[1] and isinstance(a[1][0], tuple) else _colour(E, eid)
    return out


def face_outward_normal(E, face_id):
    """Outward unit normal of a PLANE face (part coordinates), or None if the
    face is not planar.  ADVANCED_FACE.same_sense says whether the face's
    normal agrees with the plane's axis; in a closed shell it points out of
    the solid."""
    _, fa = E[face_id]
    st, sa = E[R(fa[2])]
    if st != "PLANE":
        return None
    n = placement(E, sa[1])[:3, 2]
    return n if fa[3] == ".T." else -n


def marked_faces(E, parts):
    """Faces whose colour differs from their part's colour.
    Returns [{part, nauo, face, surface, colour, part_colour, normal, centroid_mm}],
    normal / centroid in ASSEMBLY coordinates (normal only for planar faces)."""
    col = styled_colours(E)
    out = []
    for p in parts:
        T = p["T"]
        for s in p["solids"]:
            pc = col.get(s)
            shell = R(E[s][1][1])
            for f in refs_in(E[shell][1][1]):
                if E[f][0] != "ADVANCED_FACE" or f not in col or col[f] == pc:
                    continue
                n = face_outward_normal(E, f)
                # face centroid from its vertices (good enough to say where it is)
                pts = []
                stack = list(refs_in(E[f][1][1]))
                seen = set()
                while stack:                               # vertices of the face's edges
                    e = stack.pop()
                    if e in seen:
                        continue
                    seen.add(e)
                    t, a = E[e]
                    if t == "VERTEX_POINT":
                        pts.append(point(E, a[1]))
                    elif t in ("FACE_OUTER_BOUND", "FACE_BOUND", "EDGE_LOOP", "ORIENTED_EDGE", "EDGE_CURVE"):
                        stack.extend(r for r in refs_in(a) if E[r][0] in
                                     ("EDGE_LOOP", "ORIENTED_EDGE", "EDGE_CURVE", "VERTEX_POINT"))
                ctr = np.mean(pts, axis=0) if pts else np.zeros(3)
                out.append({"part": p["name"], "nauo": p["nauo"], "face": f,
                            "surface": E[R(E[f][1][2])][0], "colour": col[f], "part_colour": pc,
                            "normal": None if n is None else (T[:3, :3] @ n).tolist(),
                            "centroid_mm": ((T[:3, :3] @ ctr + T[:3, 3]) * 1000).tolist()})
    return out


def ground_up(E, parts, head_to_knee=None):
    """'Up' (unit vector, assembly coords) from the marked ground-parallel face.
    The face only fixes the plane, so the sign is chosen to make the femur
    hang DOWN (head -> knee has a negative 'up' component) when head_to_knee
    is given; otherwise up = -(outward normal), i.e. the face rests ON the
    ground.  Raises if there is not exactly one marked planar face."""
    planar = [m for m in marked_faces(E, parts) if m["normal"] is not None]
    if len(planar) != 1:
        raise ValueError(f"expected exactly one marked planar face, found {len(planar)}: "
                         f"{[(m['part'], m['surface']) for m in marked_faces(E, parts)]}")
    n = np.array(planar[0]["normal"])
    up = -n
    if head_to_knee is not None and np.dot(up, head_to_knee) > 0:
        up = -up
    return up, planar[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--tol", type=float, default=1.0, help="mm: axis passes 'through the centre' if closer than this")
    ap.add_argument("--json", help="write the extracted data here")
    ap.add_argument("--colours", action="store_true",
                    help="list part colours and faces coloured differently (e.g. the ground-parallel face)")
    args = ap.parse_args()

    E = load(Path(args.step))
    parts = assembly(E)
    if args.colours:
        col = styled_colours(E)
        print("part colours (RGB 0..1):")
        for p in parts:
            print(f"  {p['nauo']:7s} {p['name']:20s} {[col.get(s) for s in p['solids']]}")
        mk = marked_faces(E, parts)
        print(f"\nfaces coloured differently from their part: {len(mk)}")
        for m in mk:
            print(f"  {m['part']:20s} face #{m['face']} {m['surface']:22s} colour {m['colour']} "
                  f"(part {m['part_colour']})  centroid {np.round(m['centroid_mm'], 1)} mm  "
                  f"outward normal {None if m['normal'] is None else np.round(m['normal'], 4)}")
        return
    data = {}
    for prt in parts:
        cyl, sph = part_features(E, prt)
        data[prt["nauo"]] = {"name": prt["name"], "T": prt["T"], "axes": merge_axes(cyl), "spheres": sph}

    print(f"{len(parts)} parts")
    for k, v in data.items():
        print(f"  {k:7s} {v['name']:20s} axes {len(v['axes']):3d}  spheres {len(v['spheres'])}")

    # centre of rotation: candidate = femoral-head sphere(s)
    head = [v for v in data.values() if v["name"] == "femur head"]
    print("\nfemur head spheres (centre mm, radius mm):")
    for c, r in (head[0]["spheres"] if head else []):
        print(f"   {np.round(c, 3)}  r={r:.3f}")

    if args.json:
        out = {k: {"name": v["name"], "T": v["T"].tolist(),
                   "axes": [{"p": g["p"].tolist(), "d": g["d"].tolist(), "radii": sorted(g["radii"]), "n": g["n"]}
                            for g in v["axes"]],
                   "spheres": [{"c": c.tolist(), "r": r} for c, r in v["spheres"]]}
               for k, v in data.items()}
        Path(args.json).write_text(json.dumps(out, indent=1))
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
mesh_audit.py -- Phase 2a: interrogate exported STLs before writing any URDF.

WHY THIS EXISTS
---------------
A URDF is a statement about *frames*. Every <visual> and <collision> mesh is
placed by an <origin> transform relative to its link frame, and every link frame
is placed by a <joint> relative to its parent. If you do not know, numerically,
where each STL's geometry sits relative to its own origin, you cannot write a
correct <origin> -- you can only guess and nudge, which is how teams lose three
days in RViz.

CAD exporters (AutoCAD especially) almost always write STLs in ASSEMBLY
coordinates: every part carries the offset it had in the assembled model. The
URDF needs each part expressed about ITS OWN JOINT AXIS. This script measures
the discrepancy so we can correct it deterministically.

It also answers three other questions that bite later:
  * Units.        STL has no unit metadata. None. It is a bag of floats.
                  Gazebo assumes metres. AutoCAD usually exports millimetres.
                  A 1000x scale error makes a robot the size of a building.
  * Watertight.   Inertia and volume are only defined for a closed manifold.
                  A leaky mesh gives you a meaningless (often negative) inertia
                  tensor, which Gazebo will happily accept and then explode on.
  * Complexity.   Collision meshes with >2k triangles will destroy your physics
                  step rate. 6 legs x 3 links means this multiplies by 18.

USAGE (in WSL)
--------------
    pip3 install trimesh numpy
    python3 tools/mesh_audit.py src/hexapod_description/meshes/visual
"""

import sys
import pathlib

try:
    import numpy as np
    import trimesh
except ImportError:
    sys.exit("Missing deps. Run:  pip3 install trimesh numpy")


def classify_units(max_extent_mm_guess: float) -> str:
    """
    Heuristic unit inference from raw bounding-box magnitude.

    A hexapod leg segment is physically ~30-150 mm = 0.03-0.15 m.
    So if the largest raw number is in the hundreds, the file is in mm.
    If it is a fraction of 1, the file is already in metres.
    This is a heuristic, NOT a fact -- confirm against your CAD dimensions.
    """
    e = max_extent_mm_guess
    if e > 20.0:
        return "millimetres (scale by 0.001)"
    if 0.01 <= e <= 3.0:
        return "metres (scale by 1.0)"
    if e < 0.01:
        return "SUSPICIOUS - sub-centimetre. Check export settings."
    return "ambiguous - confirm manually"


def audit(path: pathlib.Path) -> dict:
    # ------------------------------------------------------------------
    # process=True is ESSENTIAL, and the reason is a property of the STL
    # format itself.
    #
    # STL has no vertex index. It stores three full XYZ triplets per facet,
    # so a cube's 12 triangles are written as 36 independent vertices even
    # though the cube has 8 corners. Loaded verbatim, the mesh is a "triangle
    # soup": geometrically correct, topologically disconnected. Nothing knows
    # which facets share an edge.
    #
    # Watertightness is a TOPOLOGICAL property -- "does every edge belong to
    # exactly two faces". On a triangle soup the answer is always no, and
    # volume (which needs a closed surface via the divergence theorem) is
    # always nan. You would condemn every mesh you own on a false positive.
    #
    # process=True merges coincident vertices first, reconstructing the
    # adjacency, so the watertight test measures the actual part.
    #
    # Diagnostic: if vertices == 3 * faces exactly, welding did NOT happen.
    # ------------------------------------------------------------------
    raw = trimesh.load_mesh(path, process=False)
    raw_vertices = len(raw.vertices) if not isinstance(raw, trimesh.Scene) else -1

    mesh = trimesh.load_mesh(path, process=True)

    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values()]
        )

    lo, hi = mesh.bounds
    extents = mesh.extents
    max_extent = float(np.max(extents))

    # Distance from the file's origin (0,0,0) to the geometric centre of the
    # bounding box. This is the number that tells you whether the part was
    # exported about its own datum or in assembly coordinates.
    bbox_centre = (lo + hi) / 2.0
    origin_offset = float(np.linalg.norm(bbox_centre))

    # Which local axis is the part's long axis? For a leg segment this should
    # be X, matching the URDF convention that every segment extends along +X.
    long_axis = "XYZ"[int(np.argmax(extents))]

    return {
        "name": path.name,
        "triangles": len(mesh.faces),
        "vertices": len(mesh.vertices),
        "raw_vertices": raw_vertices,
        "welded_ratio": (raw_vertices / max(len(mesh.vertices), 1)),
        "long_axis": long_axis,
        "euler_number": int(mesh.euler_number),
        "bbox_min": lo,
        "bbox_max": hi,
        "extents": extents,
        "bbox_centre": bbox_centre,
        "origin_offset": origin_offset,
        "max_extent": max_extent,
        "watertight": bool(mesh.is_watertight),
        "winding_ok": bool(mesh.is_winding_consistent),
        "volume": float(mesh.volume) if mesh.is_watertight else float("nan"),
        "units": classify_units(max_extent),
    }


def main() -> int:
    if len(sys.argv) < 2:
        return int(bool(sys.exit(__doc__)))

    target = pathlib.Path(sys.argv[1])
    files = sorted(target.glob("*.stl")) if target.is_dir() else [target]
    files = [f for f in files if f.is_file()]

    if not files:
        print(f"No .stl files found under {target}")
        return 1

    print("=" * 78)
    print(f"MESH AUDIT  --  {len(files)} file(s)")
    print("=" * 78)

    problems = []

    for f in files:
        try:
            r = audit(f)
        except Exception as exc:                       # noqa: BLE001
            print(f"\n[{f.name}]  FAILED TO LOAD: {exc}")
            problems.append(f"{f.name}: unreadable")
            continue

        print(f"\n[{r['name']}]")
        print(f"  triangles            : {r['triangles']}")
        print(f"  vertices raw->welded : {r['raw_vertices']} -> {r['vertices']}"
              f"   (x{r['welded_ratio']:.2f} reduction)")
        print(f"  long axis            : {r['long_axis']}"
              "   (should be X for leg segments)")
        # Euler characteristic V - E + F. For a closed surface it equals
        # 2 - 2g, where g is the genus (number of through-holes/handles).
        # A solid bracket with 4 bolt holes drilled clean through has g = 4
        # and euler = -6. NEGATIVE IS NORMAL AND EXPECTED for real parts --
        # it is not a defect. Only read it alongside `watertight`: watertight
        # True with negative euler simply means "closed shell with holes".
        _g = (2 - r["euler_number"]) // 2
        print(f"  euler number         : {r['euler_number']}"
              f"   (genus ~{_g}: through-holes, normal for brackets)")
        print("  bbox min             : "
              f"[{r['bbox_min'][0]:10.4f} {r['bbox_min'][1]:10.4f} {r['bbox_min'][2]:10.4f}]")
        print("  bbox max             : "
              f"[{r['bbox_max'][0]:10.4f} {r['bbox_max'][1]:10.4f} {r['bbox_max'][2]:10.4f}]")
        print("  extents  (L x W x H) : "
              f"[{r['extents'][0]:10.4f} {r['extents'][1]:10.4f} {r['extents'][2]:10.4f}]")
        print("  bbox centre          : "
              f"[{r['bbox_centre'][0]:10.4f} {r['bbox_centre'][1]:10.4f} {r['bbox_centre'][2]:10.4f}]")
        print(f"  |offset from origin| : {r['origin_offset']:.4f}")
        print(f"  watertight           : {r['watertight']}")
        print(f"  winding consistent   : {r['winding_ok']}")
        print(f"  raw volume           : {r['volume']:.6f}")
        print(f"  inferred units       : {r['units']}")

        # --- flags ------------------------------------------------------
        if r["welded_ratio"] < 1.05:
            print("  >> WARNING: vertex welding had no effect (vertices == 3 x faces).")
            print("     The mesh is a disconnected triangle soup. Watertightness and")
            print("     volume below are MEANINGLESS. Usually means the exporter wrote")
            print("     facets at differing precisions so no two vertices coincide.")
            problems.append(f"{r['name']}: welding failed - soup")

        if not r["watertight"]:
            print("  >> WARNING: not watertight. Inertia from this mesh is invalid")
            print("     as-is. compute_inertia.py will fall back to the CONVEX HULL,")
            print("     which is acceptable for simulation but overestimates inertia")
            print("     for concave parts. To repair properly, run the STL through")
            print("     Meshlab or Blender (3D Print toolbox -> Make Manifold).")
            problems.append(f"{r['name']}: not watertight")
        if not r["winding_ok"]:
            print("  >> WARNING: inconsistent face winding. Normals/volume unreliable.")
            problems.append(f"{r['name']}: bad winding")
        if r["origin_offset"] > 0.25 * r["max_extent"]:
            print("  >> NOTE: geometry is offset from the file origin. Exported in")
            print("     assembly coordinates. We will compensate in the URDF <origin>.")
        if r["triangles"] > 20000:
            print("  >> NOTE: high triangle count. Fine for <visual>, far too heavy")
            print("     for <collision>. We will use primitives for collision.")

    print("\n" + "=" * 78)
    if problems:
        print("ISSUES REQUIRING ATTENTION:")
        for p in problems:
            print(f"  - {p}")
    else:
        print("All meshes clean.")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

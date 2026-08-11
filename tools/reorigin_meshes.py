#!/usr/bin/env python3
"""
reorigin_meshes.py : bake each STL's link frame into the mesh itself.

THE PROBLEM
===========
mesh_audit.py showed your exporter laid every part flat on a layout sheet:

    body test.stl        bbox centre (3392.56, 1640.91,  57.50) mm
    coxa new.stl         bbox centre (2542.19, 1710.11,  35.00) mm
    femur bottom.stl     bbox centre (3154.40, 1903.28,  15.85) mm
    femur top blah.stl   bbox centre (3184.77, 1826.22,   2.50) mm
    tibia final.stl      bbox centre (2478.58, 1245.79,  10.00) mm

Every part therefore sits 2.4 to 3.8 METRES from its own origin. The URDF
needs each part expressed about ITS OWN JOINT AXIS.

TWO WAYS TO FIX IT, AND WHY WE PICK THIS ONE
============================================
Option A: leave the STLs alone and put a large compensating translation in
each <visual><origin>. It works, but every origin in the URDF becomes an
unreadable magic number like "-3.09609 -1.90328 -0.01585", and any later
change to the CAD silently invalidates all of them.

Option B (this script): translate the geometry ONCE, write clean STLs, and
let the URDF origins be 0. The URDF then says what it means, and re-running
this script after a CAD change is a single command.

Option B also lets us fix the filenames. Spaces in a package:// URI break
ROS's resource_retriever. "coxa new.stl" would fail to load with an error
that points at the URDF rather than the filename.

WHAT "ANCHOR" MEANS
===================
For each axis we choose which feature of the bounding box lands on zero:

    "min"    -> the low face of the bbox goes to 0
    "center" -> the bbox centre goes to 0

For a leg segment the joint axis is at the NEAR END, on the centreline, at
mid thickness. So: x = min, y = center, z = center. For the body, base_link
sits at the geometric centre, so all three are "center".

CAVEAT WORTH STATING
====================
This anchors to the BOUNDING BOX, not to the actual pivot. If a servo horn
or bearing boss overhangs the pivot, the true axis is inset from the end
face by that overhang. The result will be visually close but not exact. We
correct it by eye in RViz, one leg at a time, using the *_mesh_nudge
properties in common_properties.xacro. Bounding box first, eye second, is
far faster than trying to derive the pivot analytically.

USAGE
=====
    python3 tools/reorigin_meshes.py \\
        src/hexapod_description/meshes/visual

Originals are left untouched. Cleaned files are written alongside them with
new names, so the operation is repeatable and non destructive.
"""

import pathlib
import sys

try:
    import numpy as np
    import trimesh
except ImportError:
    sys.exit("Missing deps. Run:  pip3 install 'trimesh==3.23.5' numpy")


# (input filename, output filename, anchor_x, anchor_y, anchor_z)
PARTS = [
    ("body test.stl",      "body.stl",        "center", "center", "center"),
    ("coxa new.stl",       "coxa.stl",        "min",    "center", "center"),
    ("femur bottom.stl",   "femur_lower.stl", "min",    "center", "center"),
    ("femur top blah.stl", "femur_upper.stl", "min",    "center", "min"),
    ("tibia final.stl",    "tibia.stl",       "min",    "center", "center"),
]


def anchor_value(lo: float, hi: float, mode: str) -> float:
    if mode == "min":
        return lo
    if mode == "max":
        return hi
    if mode == "center":
        return (lo + hi) / 2.0
    raise ValueError(f"unknown anchor mode: {mode}")


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    mesh_dir = pathlib.Path(sys.argv[1])
    if not mesh_dir.is_dir():
        sys.exit(f"Not a directory: {mesh_dir}")

    print("=" * 74)
    print("RE-ORIGIN MESHES")
    print("=" * 74)

    failures = 0

    for src_name, dst_name, ax, ay, az in PARTS:
        src = mesh_dir / src_name
        dst = mesh_dir / dst_name

        if not src.exists():
            print(f"\n[{src_name}]  NOT FOUND, skipping")
            failures += 1
            continue

        mesh = trimesh.load_mesh(src, process=True)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))

        lo, hi = mesh.bounds
        shift = -np.array([
            anchor_value(lo[0], hi[0], ax),
            anchor_value(lo[1], hi[1], ay),
            anchor_value(lo[2], hi[2], az),
        ])

        mesh.apply_translation(shift)
        mesh.export(dst)

        nlo, nhi = mesh.bounds
        print(f"\n[{src_name}]  ->  {dst_name}")
        print(f"  anchor            : x={ax}, y={ay}, z={az}")
        print(f"  translation (mm)  : "
              f"[{shift[0]:10.4f} {shift[1]:10.4f} {shift[2]:10.4f}]")
        print(f"  new bbox min      : "
              f"[{nlo[0]:10.4f} {nlo[1]:10.4f} {nlo[2]:10.4f}]")
        print(f"  new bbox max      : "
              f"[{nhi[0]:10.4f} {nhi[1]:10.4f} {nhi[2]:10.4f}]")

        # Sanity: the anchored axes must now read zero.
        checks = [
            ("x", ax, nlo[0], nhi[0]),
            ("y", ay, nlo[1], nhi[1]),
            ("z", az, nlo[2], nhi[2]),
        ]
        for axis, mode, a, b in checks:
            got = a if mode == "min" else (b if mode == "max" else (a + b) / 2.0)
            if abs(got) > 1e-3:
                print(f"  >> WARNING: {axis} anchor did not land on zero "
                      f"(got {got:.6f})")
                failures += 1

    print("\n" + "=" * 74)
    if failures:
        print(f"COMPLETED WITH {failures} ISSUE(S)")
    else:
        print("All meshes re-origined. URDF <origin> can now be 0 0 0.")
        print("Next: set use_meshes to true in common_properties.xacro")
    print("=" * 74)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

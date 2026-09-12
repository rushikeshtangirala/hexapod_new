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
# Revision 2: new leg design, original chassis.
#
#   coxa_raw.stl    61.0 x 25.0 x 51.5 mm    was 150 mm long
#   femur_raw.stl  120.0 x 25.0 x 33.0 mm    was 117 mm, now ONE part
#   tibia_raw.stl  109.8 x 61.3 x 25.3 mm    was 150 mm
#   body test.stl  312.7 x 283.3 x 115 mm    unchanged
#
# The femur is a single part now, so femur_lower/femur_upper are gone and
# leg_macro.xacro carries one femur visual instead of two.
#
# "washer test.stl" is deliberately absent: at 20 x 20 x 7 mm it is a
# fastener, not a link. Hardware that does not move relative to its parent
# belongs in the parent's mass, not in the kinematic tree. Adding a link and
# a fixed joint for every washer would inflate the tree for no kinematic
# benefit and slow the solver.
PARTS = [
    ("body test.stl", "body.stl",  "center", "center", "center"),
    ("coxa_raw.stl",  "coxa.stl",  "min",    "center", "center"),
    ("femur_raw.stl", "femur.stl", "min",    "center", "center"),
    ("tibia_raw.stl", "tibia.stl", "min",    "center", "center"),
]

# =============================================================================
# ASSEMBLY MODE  --  the correct way to do this, and the fix for orientation
# =============================================================================
# THE PROBLEM WITH LAYOUT-SHEET EXPORTS
#
# The parts above were exported individually, each laid flat on the XY plane
# in its own convenient pose. That is fine for 3D printing and useless for
# building a robot model, because it DISCARDS the relative orientation of the
# parts. Once a part has been rotated onto a layout sheet, nothing in its STL
# records which way it faced in the assembly.
#
# The consequence is that every part's roll about the leg axis has to be
# rediscovered by trial and error, one 90 degree step at a time, per part.
# That is the rpy guessing we have been doing, and it is avoidable.
#
# THE FIX: EXPORT IN ASSEMBLY POSITION
#
# Export each part WITHOUT moving it, straight out of the assembled leg, so
# every part shares one coordinate system: the assembly's. Then the relative
# orientations are already correct and are preserved automatically.
#
# All this script has to do then is a PURE TRANSLATION per part, moving each
# link's joint to its own origin:
#
#     coxa  : joint is at the assembly origin       -> shift (0, 0, 0)
#     femur : joint is L1 along the leg             -> shift (-L1, 0, 0)
#     tibia : joint is L1+L2 along the leg          -> shift (-(L1+L2), 0, 0)
#
# No rotation, no guessing, nothing to tune by eye. The orientation problem
# disappears rather than being solved.
#
# WHAT YOU MUST DO IN AUTOCAD FIRST
#   1. Open the ASSEMBLED leg, joints at zero (leg straight).
#   2. Move/rotate the whole assembly so that:
#        - the COXA JOINT AXIS sits at the origin (0, 0, 0)
#        - the leg extends along +X
#        - "up" on the robot is +Z
#      Move the assembly as one unit. Do not move parts relative to each other.
#   3. Export each part SEPARATELY, without repositioning it:
#        coxa_asm.stl   femur_asm.stl   tibia_asm.stl
#
# Then:  python3 tools/reorigin_meshes.py <dir> --assembly
# =============================================================================

# (input, output, x shift in millimetres). Y and Z are never shifted: in
# assembly coordinates the part is already laterally and vertically correct.
L1_MM = 61.0
L2_MM = 120.0

PARTS_ASSEMBLY = [
    ("coxa_asm.stl",  "coxa.stl",  0.0),
    ("femur_asm.stl", "femur.stl", -L1_MM),
    ("tibia_asm.stl", "tibia.stl", -(L1_MM + L2_MM)),
]


def anchor_value(lo: float, hi: float, mode: str) -> float:
    if mode == "min":
        return lo
    if mode == "max":
        return hi
    if mode == "center":
        return (lo + hi) / 2.0
    raise ValueError(f"unknown anchor mode: {mode}")


def run_assembly_mode(mesh_dir: pathlib.Path) -> int:
    """
    Pure translation, orientation preserved. See the long note above PARTS.

    Nothing is rotated here, deliberately. If a part looks wrong after this,
    the assembly was not aligned to +X before export, and the fix is in
    AutoCAD rather than in a fudge factor here. Baking a correction into this
    script would hide a wrong export and make the next one wrong too.
    """
    print("=" * 74)
    print("RE-ORIGIN, ASSEMBLY MODE  (translation only, orientation preserved)")
    print("=" * 74)

    failures = 0
    for src_name, dst_name, x_shift in PARTS_ASSEMBLY:
        src = mesh_dir / src_name
        if not src.exists():
            print(f"\n[{src_name}]  NOT FOUND")
            print("   Export it from the assembled leg without repositioning.")
            failures += 1
            continue

        mesh = trimesh.load_mesh(src, process=True)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))

        before_lo, before_hi = mesh.bounds
        mesh.apply_translation([x_shift, 0.0, 0.0])
        lo, hi = mesh.bounds
        mesh.export(mesh_dir / dst_name)

        print(f"\n[{src_name}]  ->  {dst_name}")
        print(f"  x shift           : {x_shift:+.1f} mm")
        print(f"  bbox before x     : [{before_lo[0]:9.3f} {before_hi[0]:9.3f}]")
        print(f"  bbox after  x     : [{lo[0]:9.3f} {hi[0]:9.3f}]")
        print(f"  bbox after  y     : [{lo[1]:9.3f} {hi[1]:9.3f}]")
        print(f"  bbox after  z     : [{lo[2]:9.3f} {hi[2]:9.3f}]")

        # The joint should now sit at x = 0, so the part should start at or
        # very near zero and extend in +x. A large negative minimum means the
        # link length constants do not match the real assembly.
        if lo[0] < -5.0:
            print(f"  >> WARNING: extends {abs(lo[0]):.1f} mm behind its joint.")
            print("     Either that is real (a bracket wrapping the pivot), or")
            print("     L1/L2 in this file do not match the assembly. Check the")
            print("     axis-to-axis distances.")

        # In assembly coordinates the leg lies along X, so a part whose Y or Z
        # extent exceeds its X extent is either genuinely stubby (the coxa is)
        # or was not aligned before export.
        ext = hi - lo
        if ext[0] < max(ext[1], ext[2]) and dst_name != "coxa.stl":
            print("  >> WARNING: this part is not longest along X. The assembly")
            print("     was probably not aligned to +X before export.")
            failures += 1

    print("\n" + "=" * 74)
    if failures:
        print(f"COMPLETED WITH {failures} ISSUE(S)")
    else:
        print("Done. Set every *_mesh_rpy back to '0 0 0': assembly-position")
        print("exports carry the correct orientation already.")
    print("=" * 74)
    return 1 if failures else 0


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    mesh_dir = pathlib.Path(sys.argv[1])
    if not mesh_dir.is_dir():
        sys.exit(f"Not a directory: {mesh_dir}")

    if "--assembly" in sys.argv:
        return run_assembly_mode(mesh_dir)

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

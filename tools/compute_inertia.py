#!/usr/bin/env python3
"""
compute_inertia.py -- Phase 3: derive URDF <inertial> blocks from STL geometry.

WHY WE DO NOT USE AUTOCAD MASSPROP
----------------------------------
Three independent reasons:
  1. MASSPROP reports the inertia tensor about the WCS origin (or the current
     UCS), not about the part's own centre of mass. URDF requires the tensor
     about the COM, expressed in a frame whose origin is given separately by
     <origin xyz="...">. Feeding MASSPROP numbers straight into URDF is a
     silent, large error.
  2. MASSPROP needs a 3D SOLID. STL export flattens solids to a triangle shell.
     If the solid is gone, the number is gone.
  3. It is not reproducible. A script that regenerates every inertia from the
     mesh is auditable and re-runnable when the CAD changes -- which it will.

WHAT AN INERTIA TENSOR IS, AND WHY GAZEBO NEEDS IT
--------------------------------------------------
Mass tells you how a body resists linear acceleration (F = ma). The inertia
tensor is the rotational analogue: it tells you how the body resists ANGULAR
acceleration, and crucially that the resistance depends on the AXIS you spin
it about. It is a symmetric 3x3:

        [ Ixx  Ixy  Ixz ]
    I = [ Ixy  Iyy  Iyz ]        (only 6 unique values -> the 6 URDF attributes)
        [ Ixz  Iyz  Izz ]

Diagonal terms are the second moments of mass about each axis. Off-diagonal
"products of inertia" describe mass asymmetry -- they are what makes a spinning
body wobble. A femur link is long and thin, so its Ixx (about its long axis)
is much smaller than Iyy/Izz. Get this wrong and the leg's swing dynamics are
wrong, which means your PID gains from Phase 5 are tuned against a fiction.

WHY BAD INERTIA IS THE #1 CAUSE OF "MY ROBOT EXPLODES IN GAZEBO"
----------------------------------------------------------------
Gazebo's ODE solver inverts the mass matrix every step. If a link's inertia is
too small relative to its neighbours (the classic case: someone copy-pasted
ixx=iyy=izz=0.001 into all 19 links), the mass ratio across a joint becomes
enormous, the system becomes numerically stiff, and the solver diverges. You
see the robot vibrate, then launch into orbit. It is never a "Gazebo bug".

VALIDATION THIS SCRIPT PERFORMS
-------------------------------
  * Positive definiteness: all three principal moments (eigenvalues) > 0.
  * Triangle inequality:   I1 + I2 >= I3 for every permutation. This is a hard
                           physical law for any real rigid body. A tensor that
                           violates it does not correspond to any distribution
                           of mass in 3D space, and no amount of PID tuning
                           will rescue a sim built on one.

USAGE
-----
    # By material density (preferred -- physically grounded):
    python3 tools/compute_inertia.py femur.stl --scale 0.001 --density 1240

    # By measured mass (better still if you have a scale and printed parts):
    python3 tools/compute_inertia.py femur.stl --scale 0.001 --mass 0.045

COMMON DENSITIES  [kg/m^3]
    PLA 1240 | PETG 1270 | ABS 1040 | Nylon 1150
    Aluminium 6061 2700 | Carbon-fibre plate ~1600 | Acrylic 1180
NOTE: FDM prints are not solid. At 30% infill the effective density is roughly
0.4-0.5x the bulk figure (walls are solid, interior is not). If you printed the
parts, WEIGH ONE and use --mass. That single measurement beats any estimate.
"""

import argparse
import sys

try:
    import numpy as np
    import trimesh
except ImportError:
    sys.exit("Missing deps. Run:  pip3 install trimesh numpy")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="Multiply mesh units to reach METRES. mm -> 0.001")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--density", type=float, help="kg/m^3")
    g.add_argument("--mass", type=float, help="kg (measured; overrides density)")
    ap.add_argument("--link-name", default=None, help="For the emitted comment")
    args = ap.parse_args()

    mesh = trimesh.load_mesh(args.mesh, process=True)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(list(mesh.geometry.values()))

    # Scale into metres FIRST. Inertia scales as L^5 (volume L^3 x length^2),
    # so scaling afterwards is a five-fold error waiting to happen.
    if args.scale != 1.0:
        mesh.apply_scale(args.scale)

    if not mesh.is_watertight:
        print("WARNING: mesh is not watertight. Falling back to its CONVEX HULL.",
              file=sys.stderr)
        print("         This OVERESTIMATES inertia for concave parts. Acceptable",
              file=sys.stderr)
        print("         for simulation; note it in your report.", file=sys.stderr)
        mesh = mesh.convex_hull

    volume = float(mesh.volume)
    if volume <= 0:
        sys.exit("ERROR: non-positive volume. Mesh normals are inverted.")

    if args.mass is not None:
        density = args.mass / volume
        mass = args.mass
    else:
        density = args.density
        mass = density * volume

    mesh.density = density
    com = mesh.center_mass                 # metres, in the mesh's own frame
    inertia = mesh.moment_inertia          # about the COM -- exactly what URDF wants

    # ---- validation -----------------------------------------------------
    eigs = np.linalg.eigvalsh(inertia)
    ok_pd = bool(np.all(eigs > 0))
    I1, I2, I3 = sorted(eigs)
    ok_tri = (I1 + I2) >= I3 * (1.0 - 1e-9)

    print(f"\nmesh            : {args.mesh}")
    print(f"scale to metres : {args.scale}")
    print(f"volume          : {volume:.9f} m^3   ({volume * 1e6:.2f} cm^3)")
    print(f"density         : {density:.2f} kg/m^3")
    print(f"mass            : {mass:.6f} kg")
    print(f"centre of mass  : [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}] m")
    print(f"principal moments: {eigs}")
    print(f"positive definite : {'PASS' if ok_pd else 'FAIL'}")
    print(f"triangle ineq.    : {'PASS' if ok_tri else 'FAIL'}")

    if not (ok_pd and ok_tri):
        print("\n>> DO NOT USE THIS TENSOR. Repair the mesh first "
              "(check normals / watertightness).", file=sys.stderr)

    name = args.link_name or args.mesh
    print("\n<!-- generated by tools/compute_inertia.py from "
          f"{name}, density {density:.1f} kg/m^3 -->")
    print("<inertial>")
    print(f'  <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>')
    print(f'  <mass value="{mass:.6f}"/>')
    print(f'  <inertia ixx="{inertia[0][0]:.9f}" ixy="{inertia[0][1]:.9f}" '
          f'ixz="{inertia[0][2]:.9f}"')
    print(f'           iyy="{inertia[1][1]:.9f}" iyz="{inertia[1][2]:.9f}"')
    print(f'           izz="{inertia[2][2]:.9f}"/>')
    print("</inertial>")
    return 0 if (ok_pd and ok_tri) else 2


if __name__ == "__main__":
    raise SystemExit(main())

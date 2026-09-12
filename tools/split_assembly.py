#!/usr/bin/env python3
"""
split_assembly.py : recover per-part meshes, in assembly pose, from a single
                    assembled-leg STL.

=============================================================================
WHY THIS EXISTS
=============================================================================
The individual part STLs were exported laid out flat on a layout sheet, each
rotated into its own convenient pose. That is correct for 3D printing and
useless for a robot model, because it DESTROYS the relative orientation of
the parts. Nothing in an individually-exported STL records which way the part
faced in the assembly, so every part's roll has to be rediscovered by trial
and error. That is a search we should not be running at all.

An ASSEMBLED STL still contains it. The parts are disjoint solids, so
trimesh.split() recovers each as its own mesh, still sitting exactly where
the assembly put it. Relative orientation comes back for free.

=============================================================================
WHAT THIS SCRIPT DOES
=============================================================================
 1. Splits the assembly into connected components.
 2. Identifies each by VOLUME, matched against the parts we already measured:
        bracket  17 155 mm^3     (appears 3x: one per joint)
        femur    49 985 mm^3
        tibia    28 098 mm^3
        washer      993 mm^3     (ignored: hardware, not a link)
 3. Finds the leg's long axis and rotates the WHOLE assembly, rigidly, so the
    leg lies along +X with the coxa end at the low end. Rotating everything
    together preserves the relative orientations, which is the entire point.
 4. Groups the components into three links:
        coxa  = the proximal bracket
        femur = femur member + the middle bracket
        tibia = tibia truss  + the distal bracket
    A bracket bolted to a link is part of that link. It has no degree of
    freedom, so it is geometry, not a body.
 5. Translates each link so its own joint sits at the origin, and writes
    coxa.stl, femur.stl, tibia.stl.

The result needs NO rpy correction. Set every *_mesh_rpy to "0 0 0".

=============================================================================
USAGE
=============================================================================
    python3 tools/split_assembly.py "<dir>/leg assembled.stl"

Add --dry-run to inspect the components without writing anything. Do that
first: the component table tells you whether the split found what we expect.
"""

from __future__ import annotations

import pathlib
import sys

try:
    import numpy as np
    import trimesh
except ImportError:
    sys.exit("Missing deps. Run:  pip3 install 'trimesh==3.23.5' numpy")


# Reference volumes in mm^3, from mesh_audit.py on the individual parts.
# Matching on volume is robust because it is invariant to rotation and
# translation, which is exactly what we do not know yet.
REF = {
    "bracket": 17155.0,
    "femur":   49985.0,
    "tibia":   28098.0,
    "washer":    993.0,
}
TOL = 0.15          # 15% tolerance; STL tessellation differs between exports

L1_MM = 61.0        # coxa  joint -> femur joint
L2_MM = 120.0       # femur joint -> tibia joint


def identify(volume: float) -> str:
    best, best_err = "unknown", 1e9
    for name, ref in REF.items():
        err = abs(volume - ref) / ref
        if err < best_err:
            best, best_err = name, err
    return best if best_err <= TOL else f"unknown({volume:.0f})"


def patch_project(root: pathlib.Path, l1: float, l2: float, l3: float) -> int:
    """
    Write the measured lengths into the two files that must agree, and turn
    off the settings that the assembly export makes redundant.

    Done in code rather than by hand because the same three numbers live in
    two files, and a mismatch between them does not error: it silently makes
    the IK solve for a robot that is not the one being drawn. That class of
    bug has cost this project real time already.

    Every substitution is checked for exactly one match. A pattern that
    matches zero or many times means the file has moved on and the edit is
    refused rather than applied blindly.
    """
    import re

    xacro = root / "src/hexapod_description/urdf/common_properties.xacro"
    kin = root / "src/hexapod_gait/hexapod_gait/kinematics.py"

    for f in (xacro, kin):
        if not f.exists():
            print(f"  >> NOT FOUND: {f}")
            return 1

    edits = 0

    def sub_once(text: str, pattern: str, repl: str, label: str) -> str:
        nonlocal edits
        new, n = re.subn(pattern, repl, text, count=0)
        if n != 1:
            print(f"  >> {label}: expected 1 match, found {n}. SKIPPED.")
            return text
        print(f"     {label}")
        edits += 1
        return new

    print("\n  patching common_properties.xacro")
    t = xacro.read_text(encoding="utf-8")
    for name, val in (("coxa_length", l1), ("femur_length", l2),
                      ("tibia_length", l3)):
        t = sub_once(
            t,
            rf'(<xacro:property name="{name}"\s+value=")[^"]*(")',
            rf'\g<1>{val/1000:.4f}\g<2>',
            f"{name} -> {val/1000:.4f} m",
        )
    # Each link mesh now contains its own bracket, so drawing a separate one
    # would render it twice.
    t = sub_once(
        t,
        r'(<xacro:property name="show_joint_brackets" value=")[^"]*(")',
        r'\g<1>false\g<2>',
        "show_joint_brackets -> false",
    )
    # Assembly-position exports already carry the correct orientation.
    for arg in ("coxa_rpy", "femur_rpy", "tibia_rpy"):
        t = sub_once(
            t,
            rf'(<xacro:arg name="{arg}"\s+default=")[^"]*(")',
            r'\g<1>0 0 0\g<2>',
            f"{arg} -> 0 0 0",
        )
    xacro.write_text(t, encoding="utf-8")

    print("\n  patching kinematics.py")
    k = kin.read_text(encoding="utf-8")
    k = sub_once(
        k,
        r'^L1, L2, L3 = .*$',
        f"L1, L2, L3 = {l1/1000:.4f}, {l2/1000:.4f}, {l3/1000:.4f}",
        "L1, L2, L3",
    )
    kin.write_text(k, encoding="utf-8")

    print(f"\n  {edits} edit(s) applied")
    return 0


def three_link_mode(found: list, scene, out_dir: pathlib.Path, dry: bool,
                    patch_dir: pathlib.Path | None = None) -> int:
    """
    Three components == three links. Order them along the leg and export.

    Also MEASURES the link lengths, which is the part worth having. Until
    now coxa/femur/tibia lengths have been bounding box values, and a
    bounding box measures the PART while the URDF needs the distance between
    JOINT AXES. Those differ wherever a horn or boss overhangs the pivot.

    Here we can do better. Two links that share a joint overlap in X, because
    the bracket of one wraps the pivot of the other. The centre of that
    overlap is the joint. So the joint positions, and therefore the true
    axis-to-axis lengths, fall straight out of the assembly geometry.
    """
    parts = sorted(found, key=lambda f: f["centroid"][0])
    names = ["coxa", "femur", "tibia"]

    # =====================================================================
    # WHICH COMPONENT IS WHICH LINK: STATED, NOT GUESSED.
    #
    # An earlier version tried to work this out from geometry, on the theory
    # that the coxa is the shortest link. That heuristic was wrong, because
    # each component is an ASSEMBLY (member plus brackets), not a bare part,
    # so the spans do not rank the way the part lengths do.
    #
    # It produced a robot built backwards: tibia bolted to the chassis, coxa
    # at the foot. Everything downstream still passed, because a reversed leg
    # is kinematically self-consistent. It is simply the wrong machine.
    #
    # A wrong guess that yields a plausible robot is worse than no guess, so
    # this is now DECLARED. The default is the observed CAD layout:
    #
    #     component at LOW X   = tibia   (foot end)
    #     component in MIDDLE  = femur
    #     component at HIGH X  = coxa    (body end)
    #
    # Override with --order if a future export is laid out differently:
    #     --order coxa,femur,tibia
    # =====================================================================
    order = ["tibia", "femur", "coxa"]
    if "--order" in sys.argv:
        order = [s.strip() for s in
                 sys.argv[sys.argv.index("--order") + 1].split(",")]
    if sorted(order) != ["coxa", "femur", "tibia"]:
        print(f"\n  >> --order must name each of coxa, femur, tibia. Got: {order}")
        return 1

    def span_x(p) -> float:
        lo, hi = p["mesh"].bounds
        return float(hi[0] - lo[0])

    print("\n  component to link mapping, along +X (low to high):")
    for nm, p in zip(order, parts):
        print(f"    x = {p['centroid'][0]:8.2f}  span {span_x(p):6.1f} mm  ->  {nm}")

    # The URDF needs the coxa at the origin with the leg extending along +X.
    # If the declared order puts the coxa at the far end, spin the WHOLE
    # assembly 180 degrees about Z.
    #
    # A ROTATION, not a mirror. A reflection would invert the handedness and
    # silently turn every left leg into a right one, which looks fine in RViz
    # and is wrong the moment the gait runs.
    if order[0] != "coxa":
        print("\n  Coxa is at the +X end. Rotating the assembly 180 degrees")
        print("  about Z so the coxa sits at the origin and the leg runs +X.")
        spin = trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1])
        for f in found:
            f["mesh"].apply_transform(spin)
            f["centroid"] = f["mesh"].bounds.mean(axis=0)
        scene.apply_transform(spin)
        parts = sorted(found, key=lambda f: f["centroid"][0])
        order = list(reversed(order))

    if order != ["coxa", "femur", "tibia"]:
        print(f"\n  >> After orienting, the order is {order}, not")
        print("     coxa, femur, tibia. The femur must be the middle link.")
        return 1

    names = order
    print("\n  Final ordering, proximal to distal:")
    print(f"    coxa  {span_x(parts[0]):6.1f} mm  (attaches to the body)")
    print(f"    femur {span_x(parts[1]):6.1f} mm")
    print(f"    tibia {span_x(parts[2]):6.1f} mm  (carries the foot)")
    print("\n  link    span x (mm)          span y            span z"
          "          length")
    for name, p in zip(names, parts):
        lo, hi = p["mesh"].bounds
        print(f"    {name:<6} [{lo[0]:8.2f} {hi[0]:8.2f}]  "
              f"[{lo[1]:7.2f} {hi[1]:7.2f}]  "
              f"[{lo[2]:7.2f} {hi[2]:7.2f}]  {hi[0]-lo[0]:7.2f}")

    # ---- locate the joints from the overlaps ----------------------------
    def overlap_centre(a, b) -> float:
        a_lo, a_hi = a["mesh"].bounds
        b_lo, b_hi = b["mesh"].bounds
        lo = max(a_lo[0], b_lo[0])
        hi = min(a_hi[0], b_hi[0])
        if hi < lo:                       # no overlap: use the midpoint
            return (a_hi[0] + b_lo[0]) / 2.0
        return (lo + hi) / 2.0

    coxa_joint_x = parts[0]["mesh"].bounds[0][0]      # proximal end
    femur_joint_x = overlap_centre(parts[0], parts[1])
    tibia_joint_x = overlap_centre(parts[1], parts[2])
    foot_x = parts[2]["mesh"].bounds[1][0]            # distal tip

    l1 = femur_joint_x - coxa_joint_x
    l2 = tibia_joint_x - femur_joint_x
    l3 = foot_x - tibia_joint_x

    print("\n  MEASURED AXIS-TO-AXIS LENGTHS  (millimetres)")
    print(f"    coxa  joint -> femur joint : {l1:7.2f}")
    print(f"    femur joint -> tibia joint : {l2:7.2f}")
    print(f"    tibia joint -> foot tip    : {l3:7.2f}")
    print("\n  Put these into common_properties.xacro, in METRES:")
    print(f'    <xacro:property name="coxa_length"  value="{l1/1000:.4f}"/>')
    print(f'    <xacro:property name="femur_length" value="{l2/1000:.4f}"/>')
    print(f'    <xacro:property name="tibia_length" value="{l3/1000:.4f}"/>')
    print("  and the matching L1, L2, L3 in hexapod_gait/kinematics.py.")

    # ---- normalise laterally and vertically -----------------------------
    # y = 0 on the leg centreline, z = 0 at the joint axis height. Applied to
    # the whole assembly, so relative orientation is untouched.
    all_lo, all_hi = scene.bounds
    y_mid = (all_lo[1] + all_hi[1]) / 2.0
    z_mid = (all_lo[2] + all_hi[2]) / 2.0

    joint_x = [coxa_joint_x, femur_joint_x, tibia_joint_x]

    # Links to turn end-for-end, baked into the mesh rather than done with a
    # URDF <origin rpy>.
    #
    # WHY IN THE MESH. A URDF rotation happens about the LINK ORIGIN, which
    # is the joint, so a 180 degree flip throws the part to the far side of
    # its own joint and it must be translated back by exactly its own length.
    # Get that number slightly wrong and the part floats off into space,
    # which is precisely what kept happening.
    #
    # Rotating about the part's OWN CENTRE has no such coupling: the bounding
    # box is unchanged, so the part stays exactly where it sat and simply
    # faces the other way. One operation, no compensating translation, nothing
    # to tune.
    #
    #   --flip tibia          or   --flip tibia,coxa
    flip = set()
    if "--flip" in sys.argv:
        flip = {s.strip() for s in
                sys.argv[sys.argv.index("--flip") + 1].split(",")}

    print("\n  writing link meshes")
    for name, p, jx in zip(names, parts, joint_x):
        m = p["mesh"]
        m.apply_translation([-jx, -y_mid, -z_mid])

        if name in flip:
            centre = m.bounds.mean(axis=0)
            m.apply_transform(trimesh.transformations.rotation_matrix(
                np.pi, [0, 1, 0], point=centre))
            print(f"    {name:<6} flipped 180 deg about Y, in place")
        lo, hi = m.bounds
        print(f"    {name:<6} x [{lo[0]:7.2f} {hi[0]:7.2f}]  "
              f"y [{lo[1]:7.2f} {hi[1]:7.2f}]  "
              f"z [{lo[2]:7.2f} {hi[2]:7.2f}]")
        if not dry:
            m.export(out_dir / f"{name}.stl")
            # Also write straight to the Windows side, which is the source of
            # truth. Writing only into the WSL mirror would be undone by the
            # next hexsync, silently reverting the work.
            if patch_dir is not None:
                mesh_out = patch_dir / "src/hexapod_description/meshes/visual"
                mesh_out.mkdir(parents=True, exist_ok=True)
                m.export(mesh_out / f"{name}.stl")

    if not dry and patch_dir is not None:
        patch_project(patch_dir, l1, l2, l3)

    print("\n" + "=" * 74)
    if dry:
        print("DRY RUN, nothing written. Re-run without --dry-run to export.")
    elif patch_dir is not None:
        print("Meshes exported and the project updated. Next:")
        print("    hexsync && hexbuild")
        print("    ros2 launch hexapod_description display.launch.py")
    else:
        print("Wrote coxa.stl, femur.stl, tibia.stl in assembly orientation.")
        print("Set every *_mesh_rpy to '0 0 0' and show_joint_brackets to")
        print("false: each link mesh now already contains its own bracket.")
    print("=" * 74)
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    src = pathlib.Path(sys.argv[1])
    dry = "--dry-run" in sys.argv
    if not src.exists():
        sys.exit(f"Not found: {src}")

    # --patch-dir points at the WINDOWS project root, the source of truth.
    # Meshes and the measured lengths are written there so the next hexsync
    # carries them into the workspace, rather than being overwritten by it.
    patch_dir = None
    if "--patch-dir" in sys.argv:
        patch_dir = pathlib.Path(sys.argv[sys.argv.index("--patch-dir") + 1])
        if not patch_dir.is_dir():
            sys.exit(f"--patch-dir not a directory: {patch_dir}")

    out_dir = src.parent

    scene = trimesh.load_mesh(src, process=True)
    if isinstance(scene, trimesh.Scene):
        scene = trimesh.util.concatenate(list(scene.geometry.values()))

    print("=" * 74)
    print(f"ASSEMBLY SPLIT : {src.name}")
    print("=" * 74)
    print(f"  triangles {len(scene.faces)}, extents "
          f"{np.round(scene.extents, 2)}")

    parts = scene.split(only_watertight=False)
    print(f"  split into {len(parts)} connected component(s)")
    if len(parts) < 2:
        print("\n  >> Only one component. The exporter merged the solids into a")
        print("     single shell, so they cannot be separated geometrically.")
        print("     Re-export with parts kept as separate solids, or export")
        print("     each part individually IN ASSEMBLY POSITION.")
        return 1

    # ---- classify -------------------------------------------------------
    found = []
    for i, p in enumerate(parts):
        vol = abs(float(p.volume)) if p.is_watertight else abs(
            float(p.convex_hull.volume))
        found.append({"idx": i, "mesh": p, "vol": vol,
                      "kind": identify(vol),
                      "centroid": p.bounds.mean(axis=0)})

    print("\n  component      kind        volume mm^3     centroid")
    for f in found:
        c = f["centroid"]
        print(f"    {f['idx']:<3}       {f['kind']:<10}  {f['vol']:12.1f}   "
              f"[{c[0]:8.2f} {c[1]:8.2f} {c[2]:8.2f}]")

    # =====================================================================
    # THREE COMPONENTS: treat them as the three links, ordered along the leg.
    #
    # This path exists because volume matching failed, and the reason it
    # failed is informative. trimesh.split() groups faces that are
    # CONNECTED. Parts bolted together inside a link touch each other, so
    # they come back as ONE component; parts across a joint have running
    # clearance, so they stay separate.
    #
    # That means the components are not parts at all. They are LINKS, which
    # is exactly what we want and better than what the volume table was
    # looking for. Only component 0 matched a reference volume (49990.6 vs
    # the femur member's 49985.4, 0.01% apart) because the femur happens to
    # be a single part with nothing bolted to it in this assembly.
    #
    # Position along the leg is the reliable discriminator: proximal is the
    # coxa, distal is the tibia. It needs no reference data at all, so it
    # survives any CAD revision.
    # =====================================================================
    if len(parts) == 3:
        return three_link_mode(found, scene, out_dir, dry, patch_dir)

    # ---- find the leg axis, rigidly ------------------------------------
    # Rotate the WHOLE assembly, never a single part. A rigid transform on
    # everything preserves every relative orientation; rotating parts
    # individually is what lost the information in the first place.
    long_axis = int(np.argmax(scene.extents))
    axis_name = "XYZ"[long_axis]
    print(f"\n  leg long axis: {axis_name}")

    T = np.eye(4)
    if long_axis == 1:            # Y -> X
        T = trimesh.transformations.rotation_matrix(-np.pi / 2, [0, 0, 1])
    elif long_axis == 2:          # Z -> X
        T = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])

    for f in found:
        f["mesh"].apply_transform(T)
        f["centroid"] = f["mesh"].bounds.mean(axis=0)

    femur = next((f for f in found if f["kind"] == "femur"), None)
    tibia = next((f for f in found if f["kind"] == "tibia"), None)
    brackets = sorted((f for f in found if f["kind"] == "bracket"),
                      key=lambda f: f["centroid"][0])

    if femur is None or tibia is None:
        print("\n  >> Could not identify the femur and tibia by volume.")
        print("     Check the table above against the reference volumes.")
        return 1

    # Distal is whichever end the tibia is at. If the tibia sits at low X,
    # the assembly points the wrong way, so spin it 180 about Z.
    if tibia["centroid"][0] < femur["centroid"][0]:
        print("  coxa end was at +X; rotating 180 degrees about Z")
        flip = trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1])
        for f in found:
            f["mesh"].apply_transform(flip)
            f["centroid"] = f["mesh"].bounds.mean(axis=0)
        brackets = sorted((f for f in found if f["kind"] == "bracket"),
                          key=lambda f: f["centroid"][0])

    print(f"\n  brackets found: {len(brackets)} (expected 3)")
    for b in brackets:
        print(f"    at x = {b['centroid'][0]:8.2f}")

    # ---- group into links ----------------------------------------------
    groups: dict[str, list] = {"coxa": [], "femur": [], "tibia": []}
    if len(brackets) >= 1:
        groups["coxa"].append(brackets[0]["mesh"])
    if len(brackets) >= 2:
        groups["femur"].append(brackets[1]["mesh"])
    if len(brackets) >= 3:
        groups["tibia"].append(brackets[2]["mesh"])
    groups["femur"].append(femur["mesh"])
    groups["tibia"].append(tibia["mesh"])

    # ---- shift each link so its own joint is at the origin --------------
    shifts = {"coxa": 0.0, "femur": -L1_MM, "tibia": -(L1_MM + L2_MM)}

    print("\n  writing link meshes")
    for name, meshes in groups.items():
        if not meshes:
            print(f"    {name}: NO GEOMETRY, skipped")
            continue
        merged = trimesh.util.concatenate(meshes)
        # The joint is at the link's proximal end. Put that end at x = 0,
        # then apply the nominal joint offset.
        merged.apply_translation([-merged.bounds[0][0] + 0.0, 0, 0])
        lo, hi = merged.bounds
        print(f"    {name:<6} {len(meshes)} part(s)  "
              f"x [{lo[0]:7.2f} {hi[0]:7.2f}]  "
              f"y [{lo[1]:7.2f} {hi[1]:7.2f}]  "
              f"z [{lo[2]:7.2f} {hi[2]:7.2f}]")
        if not dry:
            merged.export(out_dir / f"{name}.stl")

    print("\n" + "=" * 74)
    if dry:
        print("DRY RUN, nothing written. Re-run without --dry-run to export.")
    else:
        print("Wrote coxa.stl, femur.stl, tibia.stl in assembly orientation.")
        print("Set every *_mesh_rpy back to '0 0 0' and rebuild.")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

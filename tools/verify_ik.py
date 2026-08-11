#!/usr/bin/env python3
"""
verify_ik.py : prove the inverse kinematics is correct, without ROS or Gazebo.

WHY THIS IS THE RIGHT TEST
==========================
IK correctness is a mathematical property. It does not need a simulator, a
physics engine, or a robot. Testing it through Gazebo means every failure has
a dozen possible causes; testing it here means a failure has exactly one.

THE CENTRAL IDEA: ROUND-TRIP CONSISTENCY

Forward kinematics is unambiguous and easy to get right. Inverse kinematics
is where sign errors, quadrant errors and branch errors live. So we test IK
AGAINST FK:

    sample joint angles  ->  FK  ->  foot position
                         ->  IK  ->  recovered angles
                         ->  FK  ->  recovered foot position

and require the two foot positions to agree to machine precision.

Note carefully what is compared: FOOT POSITIONS, not joint angles. Angles may
legitimately differ, because a two-link arm has two solutions (elbow up and
elbow down) that reach the identical point. Demanding angle equality would
fail on correct code. Demanding position equality tests exactly the property
we care about: does the leg go where we asked.

WHAT EACH TEST CATCHES
  round trip   : sign errors, wrong atan2 argument order, bad law-of-cosines
  branch       : the elbow-up/elbow-down choice being inconsistent
  known pose   : an overall convention flip that is self-consistent and
                 therefore invisible to the round trip alone
  workspace    : silent clamping at the reachability boundary
  body frame   : errors in the mount transform, i.e. left/right mirroring

USAGE
    python3 tools/verify_ik.py
"""

from __future__ import annotations

import itertools
import math
import os
import sys

sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "src", "hexapod_gait"),
)

from hexapod_gait.kinematics import (            # noqa: E402
    LEGS,
    UnreachableTarget,
    body_to_leg,
    forward_kinematics,
    forward_kinematics_body,
    inverse_kinematics,
    inverse_kinematics_body,
    leg_to_body,
    within_limits,
)

TOL = 1e-9
passed = 0
failed = 0


def check(condition: bool, label: str, detail: str = "") -> None:
    global passed, failed
    if condition:
        passed += 1
    else:
        failed += 1
        print(f"  FAIL  {label}")
        if detail:
            print(f"        {detail}")


def dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


print("=" * 74)
print("INVERSE KINEMATICS VERIFICATION")
print("=" * 74)

leg = LEGS[0]

# ---------------------------------------------------------------------------
print("\n1. Round trip FK -> IK -> FK over the joint-space grid")
# ---------------------------------------------------------------------------
t1_range = [math.radians(a) for a in (-45, -20, 0, 20, 45)]
t2_range = [math.radians(a) for a in (20, 50, 80, 100, 115)]
t3_range = [math.radians(a) for a in (-140, -110, -80, -50, -20)]

worst = 0.0
worst_at = None
count = 0

for t1, t2, t3 in itertools.product(t1_range, t2_range, t3_range):
    foot = forward_kinematics(leg, t1, t2, t3)
    try:
        r1, r2, r3 = inverse_kinematics(leg, foot)
    except UnreachableTarget as exc:
        check(False, "IK rejected a pose reached by FK", str(exc))
        continue
    recovered = forward_kinematics(leg, r1, r2, r3)
    err = dist(foot, recovered)
    count += 1
    if err > worst:
        worst, worst_at = err, (t1, t2, t3)

check(worst < TOL, "round-trip error within tolerance",
      f"worst {worst:.3e} m at {worst_at}")
print(f"  {count} poses tested, worst position error {worst:.3e} m")

# ---------------------------------------------------------------------------
print("\n2. Solution branch is consistently knee-up")
# ---------------------------------------------------------------------------
# Our convention takes t3 = -acos(...), so the recovered tibia angle must
# never be positive. A positive value means the branch flipped, which in
# simulation looks like one leg suddenly inverting mid-gait.
bad_branch = 0
for t1, t2, t3 in itertools.product(t1_range, t2_range, t3_range):
    foot = forward_kinematics(leg, t1, t2, t3)
    _, _, r3 = inverse_kinematics(leg, foot)
    # Branch changed to the POSITIVE root (femur reaches out, tibia hangs
    # down) to match the CAD assembly. This assertion used to demand r3 <= 0,
    # which locked in the old branch and would have quietly failed any
    # attempt to correct it.
    if r3 < -1e-9:
        bad_branch += 1
check(bad_branch == 0, "tibia solution always >= 0 (knee-down branch)",
      f"{bad_branch} poses returned the old knee-up solution")

# ---------------------------------------------------------------------------
print("\n3. Known standing pose agrees with tools/pose.sh")
# ---------------------------------------------------------------------------
# The stance angles baked into common_properties.xacro as initial_value, and
# into tools/pose.sh as the 'stand' pose:
#     femur = +0.2694 rad ( 15.44 deg), tibia = +1.1837 rad (+67.82 deg)
# Independently expected foot position: 0.28 m out, 0.18 m down.
#
# This cross-check matters because a round trip alone cannot catch an overall
# convention flip: a consistently wrong system round-trips perfectly. Here we
# compare against a number derived separately from the URDF's declared link
# lengths, so a sign or frame error shows up.
stand = forward_kinematics(leg, 0.0, 0.2694, 1.1837)
check(abs(stand[0] - 0.28) < 2e-3, "stance pose radial reach",
      f"expected 0.280, got {stand[0]:.4f}")
check(abs(stand[1]) < 1e-9, "stance pose has no lateral offset",
      f"got {stand[1]:.6f}")
check(abs(stand[2] + 0.18) < 2e-3, "stance pose foot depth",
      f"expected -0.180, got {stand[2]:.4f}")
print(f"  foot at ({stand[0]:.4f}, {stand[1]:.4f}, {stand[2]:.4f}) m")

# Round-tripping that pose must return the same angles, since it is on the
# knee-DOWN branch that inverse_kinematics now selects.
a1, a2, a3 = inverse_kinematics(leg, stand)
check(abs(a2 - 0.2694) < 1e-3 and abs(a3 - 1.1837) < 1e-3,
      "stance pose angles recovered",
      f"got femur {a2:.6f}, tibia {a3:.6f}")

# ---------------------------------------------------------------------------
print("\n4. Workspace boundary raises rather than silently clamping")
# ---------------------------------------------------------------------------
too_far = (leg.l1 + leg.l2 + leg.l3 + 0.05, 0.0, 0.0)
try:
    inverse_kinematics(leg, too_far)
    check(False, "over-extended target rejected", "no exception raised")
except UnreachableTarget:
    check(True, "over-extended target rejected")

too_close = (leg.l1, 0.0, 0.0)          # D = 0, inside the inner radius
try:
    inverse_kinematics(leg, too_close)
    check(False, "under-extended target rejected", "no exception raised")
except UnreachableTarget:
    check(True, "under-extended target rejected")

# ---------------------------------------------------------------------------
print("\n5. Body-frame transforms round trip, all six legs")
# ---------------------------------------------------------------------------
probe = (0.05, -0.02, -0.15)
for g in LEGS:
    there = body_to_leg(g, probe)
    back = leg_to_body(g, there)
    check(dist(probe, back) < TOL, f"[{g.name}] transform round trip",
          f"error {dist(probe, back):.3e}")

# ---------------------------------------------------------------------------
print("\n6. Full body-frame IK for the default stance, all six legs")
# ---------------------------------------------------------------------------
# Feet placed radially outward from each mount at the stance radius, at a
# common height. This is the pose the gait generator will centre on.
#
# STANCE CHOICE IS A DESIGN DECISION, NOT AN ARBITRARY CONSTANT
#
# The first values tried here were radius 0.30, height -0.16, which solved
# to femur = 86.8 deg against a 90 deg limit: only 3.2 deg of headroom. That
# passes a static limit check and then fails the moment the robot walks,
# because a gait sweeps the femur several degrees every cycle. ros2_control
# clamps silently at the limit, so the failure appears as a robot that limps
# on all six legs with no error anywhere.
#
# Root cause is leg proportion: coxa 0.150 and tibia 0.150 but femur only
# 0.117. A short middle link between two long ones must point almost
# straight down to produce body height, parking it near its limit.
#
# radius 0.34 / height -0.11 moves the femur to about 70 deg, giving ~20 deg
# of headroom, at the cost of a lower body. That trade is forced by the
# geometry; it is not tuning.
#
# MIN_MARGIN below turns this from a judgement call into a test.
STANCE_RADIUS = 0.28
STANCE_HEIGHT = -0.18
MIN_MARGIN = math.radians(15.0)

print(f"  stance: radius {STANCE_RADIUS:.3f} m, height {STANCE_HEIGHT:.3f} m")
print(f"  {'leg':<5} {'coxa':>9} {'femur':>9} {'tibia':>9} {'margin':>9}")
for g in LEGS:
    fx = g.mount_x + STANCE_RADIUS * math.cos(g.mount_yaw)
    fy = g.mount_y + STANCE_RADIUS * math.sin(g.mount_yaw)
    target = (fx, fy, STANCE_HEIGHT)
    try:
        j1, j2, j3 = inverse_kinematics_body(g, target)
    except UnreachableTarget as exc:
        check(False, f"[{g.name}] default stance reachable", str(exc))
        continue

    ok, why = within_limits(g, j1, j2, j3)
    check(ok, f"[{g.name}] default stance within joint limits", why)

    recovered = forward_kinematics_body(g, j1, j2, j3)
    check(dist(target, recovered) < 1e-9,
          f"[{g.name}] body-frame round trip",
          f"error {dist(target, recovered):.3e}")

    # Distance from each joint to its NEARER limit. A gait sweeps the joints
    # about this nominal stance, so a stance sitting hard against a limit is
    # unusable however valid it looks standing still.
    margin = min(
        j1 - g.coxa_min, g.coxa_max - j1,
        j2 - g.femur_min, g.femur_max - j2,
        j3 - g.tibia_min, g.tibia_max - j3,
    )
    check(margin >= MIN_MARGIN,
          f"[{g.name}] stance has >= {math.degrees(MIN_MARGIN):.0f} deg limit margin",
          f"only {math.degrees(margin):.1f} deg of travel before a joint clamps")

    print(f"  {g.name:<5} {math.degrees(j1):9.2f} {math.degrees(j2):9.2f} "
          f"{math.degrees(j3):9.2f} {math.degrees(margin):9.1f}")

# ---------------------------------------------------------------------------
print("\n" + "=" * 74)
print(f" {passed} passed, {failed} failed")
print("=" * 74)
sys.exit(1 if failed else 0)

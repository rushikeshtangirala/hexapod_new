#!/usr/bin/env python3
"""
kinematics.py : forward and inverse kinematics for one 3-DOF hexapod leg.

This module has NO ROS dependency, deliberately. Kinematics is mathematics;
tangling it with middleware makes it untestable except by running a
simulator. Here you can validate it with `python3 -c`, and the gait
generator, the teleop node and any offline analysis all import the same
functions.

=============================================================================
1. THE CHAIN, AND THE CONVENTIONS IT INHERITS
=============================================================================
Straight from leg_macro.xacro, and these must not drift apart:

    base_link
      -> coxa joint   revolute about +Z, angle t1     (yaw, swings the leg)
        -> coxa link,  length L1 along its own +X
          -> femur joint  revolute about +Y, angle t2  (lift)
            -> femur link, length L2 along its own +X
              -> tibia joint revolute about +Y, angle t3  (extend)
                -> tibia link, length L3 along its own +X
                  -> foot

Two consequences of choosing +Y for the femur and tibia axes, and they are
the source of nearly every sign error in hexapod code:

    Rotation about +Y maps +X toward -Z (right-hand rule).
    Therefore POSITIVE t2 and t3 move the foot DOWNWARD.

We keep the right-handed axis and live with the sign, rather than flipping
the axis to make angles "feel" positive-up. A left-handed frame would break
every standard rotation formula you later reach for.

=============================================================================
2. FORWARD KINEMATICS
=============================================================================
Because both the femur and tibia rotate about the SAME axis direction (+Y in
their local frames), the whole leg beyond the coxa lies in a single vertical
plane. The coxa yaw t1 selects which plane. So a 3-D problem collapses to a
2-D one, which is why an exact closed-form solution exists at all.

Work in that plane with coordinates (r, z), r measured radially outward from
the coxa axis:

    after the coxa:   (L1, 0)
    after the femur:  (L1 + L2*cos(t2),  -L2*sin(t2))
    after the tibia:  r = L1 + L2*cos(t2) + L3*cos(t2 + t3)
                      z =    - L2*sin(t2) - L3*sin(t2 + t3)

Lift back to 3-D in the leg frame:

    x = r * cos(t1)
    y = r * sin(t1)
    z = z

=============================================================================
3. INVERSE KINEMATICS
=============================================================================
Given a foot target (x, y, z) in the leg frame:

STEP 1, the coxa. It is the only joint that can produce lateral motion:

    t1 = atan2(y, x)

STEP 2, reduce to the planar 2-link problem. Move the origin to the femur
joint, which sits L1 out along the plane:

    r  = sqrt(x^2 + y^2)
    r' = r - L1
    z' = z
    D  = sqrt(r'^2 + z'^2)        distance femur joint -> foot

STEP 3, the tibia, by the law of cosines on triangle (femur joint, tibia
joint, foot) with sides L2, L3, D. The interior angle at the tibia joint is
(pi - t3), because t3 = 0 means a straight leg:

    D^2 = L2^2 + L3^2 - 2*L2*L3*cos(pi - t3)
        = L2^2 + L3^2 + 2*L2*L3*cos(t3)

    =>  cos(t3) = (D^2 - L2^2 - L3^2) / (2*L2*L3)

acos returns a value in [0, pi], so there are two valid solutions, +/-. This
is the classic elbow-up / elbow-down ambiguity. We take the NEGATIVE root:

    t3 = -acos(k)

which is the insect-like configuration where the knee is raised above the
foot. Verify against the standing pose in tools/pose.sh: t2 = +60 deg,
t3 = -30 deg. Taking the positive root would give a mechanically valid but
inverted "knee down" leg that collides with the ground during swing.

STEP 4, the femur. Two angles added:

    beta = atan2(-z', r')     elevation of the foot below horizontal
    psi  = angle between the femur and the line (femur joint -> foot),
           again by the law of cosines:

           L3^2 = L2^2 + D^2 - 2*L2*D*cos(psi)
           =>  cos(psi) = (L2^2 + D^2 - L3^2) / (2*L2*D)

    t2 = beta + psi

=============================================================================
4. REACHABILITY
=============================================================================
A solution exists only if the target is inside the annulus swept by the two
links:

    |L2 - L3|  <=  D  <=  L2 + L3

Outside that, acos receives an argument beyond [-1, 1]. Unclamped, Python
raises ValueError; naively clamped, you silently get a wrong answer at the
workspace boundary and the leg tries to tear itself off. We raise an
explicit exception instead: an unreachable foot target is a BUG IN THE GAIT,
and the gait is where it must be fixed, not papered over here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class UnreachableTarget(ValueError):
    """Raised when a foot target lies outside the leg's workspace."""


@dataclass(frozen=True)
class LegGeometry:
    """
    Link lengths and body mounting for one leg.

    MUST match common_properties.xacro. If you change a length there, change
    it here. The long-term fix is to parse the URDF at startup, which Phase 9
    should do; for now the duplication is explicit and documented rather than
    hidden.
    """

    name: str
    l1: float          # coxa length, metres
    l2: float          # femur length
    l3: float          # tibia length
    mount_x: float     # coxa axis position in base_link
    mount_y: float
    mount_z: float
    mount_yaw: float   # splay angle, radians

    # ---- joint limits, radians -------------------------------------
    # THESE MUST MATCH common_properties.xacro. They are duplicated here
    # because the maths is deliberately ROS-free and cannot read the xacro,
    # and that duplication has already bitten once: the branch change to
    # knee-down was made in the xacro first, verify_ik.py kept checking the
    # old numbers, and reported 14 failures against limits that no longer
    # existed anywhere else.
    # If you change a limit, change it in BOTH files. tools/check_consistency
    # is the right long-term answer.
    coxa_min: float = math.radians(-60.0)
    coxa_max: float = math.radians(60.0)
    # femur_max is 120, not 90. Placing a foot under the robot rather than
    # out to the side requires the femur to swing past vertical, because the
    # 0.15 m coxa puts the femur joint well outboard. See the note in
    # common_properties.xacro; it implies a servo horn offset at assembly.
    femur_min: float = math.radians(-60.0)
    femur_max: float = math.radians(90.0)
    tibia_min: float = math.radians(-60.0)
    tibia_max: float = math.radians(130.0)


# ---------------------------------------------------------------------------
# Frame transforms: base_link  <->  leg frame
# ---------------------------------------------------------------------------

def body_to_leg(geom: LegGeometry, p: tuple[float, float, float]
                ) -> tuple[float, float, float]:
    """
    Express a point given in base_link coordinates in this leg's own frame.

    The leg frame is base_link translated to the coxa mount and rotated by
    the mount yaw. Undoing that is a translation followed by a rotation of
    -mount_yaw about Z.

    This transform is the entire reason one macro serves all six legs: in the
    leg frame every leg is identical, so the IK below never needs to know
    whether it is solving for a left or a right leg.
    """
    dx = p[0] - geom.mount_x
    dy = p[1] - geom.mount_y
    dz = p[2] - geom.mount_z

    c = math.cos(-geom.mount_yaw)
    s = math.sin(-geom.mount_yaw)
    return (c * dx - s * dy, s * dx + c * dy, dz)


def leg_to_body(geom: LegGeometry, p: tuple[float, float, float]
                ) -> tuple[float, float, float]:
    """Inverse of body_to_leg."""
    c = math.cos(geom.mount_yaw)
    s = math.sin(geom.mount_yaw)
    x = c * p[0] - s * p[1]
    y = s * p[0] + c * p[1]
    return (x + geom.mount_x, y + geom.mount_y, p[2] + geom.mount_z)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(geom: LegGeometry, t1: float, t2: float, t3: float
                       ) -> tuple[float, float, float]:
    """Joint angles -> foot position, in the LEG frame."""
    r = geom.l1 + geom.l2 * math.cos(t2) + geom.l3 * math.cos(t2 + t3)
    z = -geom.l2 * math.sin(t2) - geom.l3 * math.sin(t2 + t3)
    return (r * math.cos(t1), r * math.sin(t1), z)


def forward_kinematics_body(geom: LegGeometry, t1: float, t2: float,
                            t3: float) -> tuple[float, float, float]:
    """Joint angles -> foot position, in BASE_LINK coordinates."""
    return leg_to_body(geom, forward_kinematics(geom, t1, t2, t3))


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def inverse_kinematics(geom: LegGeometry, foot: tuple[float, float, float]
                       ) -> tuple[float, float, float]:
    """
    Foot position in the LEG frame -> (coxa, femur, tibia) angles.

    Raises UnreachableTarget if no solution exists.
    """
    x, y, z = foot

    # --- coxa ----------------------------------------------------------
    t1 = math.atan2(y, x)

    # --- reduce to the planar two-link problem -------------------------
    r = math.hypot(x, y)
    rp = r - geom.l1
    zp = z
    d = math.hypot(rp, zp)

    d_min = abs(geom.l2 - geom.l3)
    d_max = geom.l2 + geom.l3
    if not (d_min - 1e-9 <= d <= d_max + 1e-9):
        raise UnreachableTarget(
            f"[{geom.name}] foot {foot} needs reach {d:.4f} m, "
            f"workspace is [{d_min:.4f}, {d_max:.4f}] m"
        )

    # --- tibia, law of cosines ------------------------------------------
    k = (d * d - geom.l2 ** 2 - geom.l3 ** 2) / (2.0 * geom.l2 * geom.l3)
    k = max(-1.0, min(1.0, k))          # guard float error only, not range
    # BRANCH CHOICE. Changed from -acos(k) to +acos(k).
    #
    # acos returns [0, pi], so t3 = +/-acos(k) are both exact solutions that
    # put the foot in the SAME place. They differ in how the leg folds, and
    # the difference is not cosmetic.
    #
    #   NEGATIVE root (what this used to do), for the 0.28 / -0.18 stance:
    #       femur  +92.9 deg   femur points almost straight DOWN
    #       tibia  -67.8 deg   tibia reaches OUT, only 25 deg below horizontal
    #       knee 136 mm horizontally INSIDE the foot
    #
    #   POSITIVE root (now):
    #       femur  +15.4 deg   femur reaches OUT, near horizontal
    #       tibia  +67.8 deg   tibia drops DOWN, 83 deg below horizontal
    #       knee 18 mm horizontally inside the foot
    #
    # WHY THE POSITIVE ROOT IS CORRECT HERE
    #
    # 1. It is the assembly in the CAD. The femur reaches outward and the
    #    tibia hangs down. tibia.stl is 150 mm long and only 20 mm tall, so
    #    the part is straight -- the downward angle in the CAD render is the
    #    JOINT being rotated, not a bend in the link.
    #
    # 2. Knee torque. The load on the tibia joint is the foot force times
    #    the HORIZONTAL distance from joint to foot. Negative root: 136 mm.
    #    Positive root: 18 mm. Standing on three legs at 10.9 N per foot
    #    that is 1.48 N*m versus 0.19 N*m -- nearly EIGHT TIMES the load for
    #    the same stance. Measured consequence of the old branch: the tibia
    #    sagged 24.3 deg under its own robot and the body sat 5 cm low.
    #
    # 3. A near-horizontal tibia is badly conditioned for holding weight.
    #    With the tibia 25 deg below horizontal, tibia angle error converts
    #    almost entirely into BODY HEIGHT error. At 83 deg it converts almost
    #    entirely into horizontal foot position, which the gait corrects for
    #    anyway.
    #
    # THE OLD JUSTIFICATION, AND WHY IT NO LONGER HOLDS
    # The previous comment argued the positive root gives an "inverted knee
    # down leg that collides with the ground during swing", and checked it
    # against tools/pose.sh at t2=+60, t3=-30. That pose belongs to the
    # abandoned 0.34 / -0.11 stance, and pose.sh no longer runs at all. In
    # the current stance BOTH roots put the knee above the foot; the
    # positive root puts it 85 mm HIGHER, so it has more ground clearance
    # during swing, not less.
    t3 = math.acos(k)                   # positive root: femur out, knee down

    # --- femur ----------------------------------------------------------
    beta = math.atan2(-zp, rp)
    cos_psi = (geom.l2 ** 2 + d * d - geom.l3 ** 2) / (2.0 * geom.l2 * d)
    cos_psi = max(-1.0, min(1.0, cos_psi))
    psi = math.acos(cos_psi)
    # Sign follows the branch: psi is the angle between the femur and the
    # line to the foot, and the femur sits on the opposite side of that line
    # for the opposite elbow. Using +psi with +acos(k) silently produces a
    # leg that does not reach the target, and every FK/IK round-trip test
    # would fail -- which is exactly what verify_ik.py is for.
    t2 = beta - psi

    return (t1, t2, t3)


def inverse_kinematics_body(geom: LegGeometry,
                            foot: tuple[float, float, float]
                            ) -> tuple[float, float, float]:
    """Foot position in BASE_LINK coordinates -> joint angles."""
    return inverse_kinematics(geom, body_to_leg(geom, foot))


def within_limits(geom: LegGeometry, t1: float, t2: float, t3: float
                  ) -> tuple[bool, str]:
    """
    Check joint limits separately from solving.

    Kept separate on purpose. A target can be geometrically reachable but
    mechanically forbidden, and those are different failures needing
    different fixes: reachability is a gait-geometry problem, limits are a
    mechanical-design or joint-range problem. Collapsing them into one
    exception loses the distinction exactly when you need it.
    """
    checks = (
        ("coxa", t1, geom.coxa_min, geom.coxa_max),
        ("femur", t2, geom.femur_min, geom.femur_max),
        ("tibia", t3, geom.tibia_min, geom.tibia_max),
    )
    for label, value, lo, hi in checks:
        if not (lo <= value <= hi):
            return False, (
                f"[{geom.name}] {label} = {math.degrees(value):.1f} deg "
                f"outside [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]"
            )
    return True, ""


# ---------------------------------------------------------------------------
# The robot: six legs, matching hexapod.urdf.xacro
# ---------------------------------------------------------------------------

L1, L2, L3 = 0.1500, 0.1166, 0.1500

MOUNT_X_FRONT, MOUNT_X_MID, MOUNT_X_REAR = 0.095, 0.000, -0.095
MOUNT_Y_FRONT, MOUNT_Y_MID, MOUNT_Y_REAR = 0.070, 0.100, 0.070
MOUNT_Z = 0.0

YAW_FRONT = math.radians(45.0)
YAW_MID = math.radians(90.0)
YAW_REAR = math.radians(135.0)


def _leg(name, mx, my, yaw) -> LegGeometry:
    return LegGeometry(name=name, l1=L1, l2=L2, l3=L3,
                       mount_x=mx, mount_y=my, mount_z=MOUNT_Z,
                       mount_yaw=yaw)


# Order matches the `joints:` list in hexapod_control/config/controllers.yaml.
# That ordering is a contract: the command array index depends on it.
LEGS: tuple[LegGeometry, ...] = (
    _leg("lf", MOUNT_X_FRONT, MOUNT_Y_FRONT, YAW_FRONT),
    _leg("lm", MOUNT_X_MID, MOUNT_Y_MID, YAW_MID),
    _leg("lr", MOUNT_X_REAR, MOUNT_Y_REAR, YAW_REAR),
    _leg("rf", MOUNT_X_FRONT, -MOUNT_Y_FRONT, -YAW_FRONT),
    _leg("rm", MOUNT_X_MID, -MOUNT_Y_MID, -YAW_MID),
    _leg("rr", MOUNT_X_REAR, -MOUNT_Y_REAR, -YAW_REAR),
)

LEG_INDEX = {leg.name: i for i, leg in enumerate(LEGS)}

#!/usr/bin/env python3
"""
gait.py : tripod gait generation. No ROS dependency, same as kinematics.py.

=============================================================================
1. WHAT A GAIT IS
=============================================================================
A gait is a rule that turns a desired BODY velocity into a foot position for
every leg at every instant. It is not a sequence of poses. Thinking of it as
a scripted animation is the mistake that produces robots that walk only at
one speed in one direction.

The rule has to satisfy two constraints simultaneously:

  STATIC STABILITY. At every instant, enough feet must be on the ground that
  the vertical projection of the centre of mass lies inside the polygon they
  form. Three non-collinear points is the minimum. A hexapod can therefore
  lift three legs at once, which is what makes the tripod gait possible and
  why hexapods are so much easier than bipeds: stability is a geometric
  property you can check, not a dynamic one you must control.

  NON-SLIP. A foot in contact must not move relative to the GROUND. Since we
  command foot positions in the BODY frame, and the body is moving, a
  stationary-on-the-ground foot must translate backwards through the body
  frame at exactly the body's velocity. Getting this wrong is why a robot
  can appear to walk while making no progress: the legs cycle, but the
  stance phase drags the feet, and friction turns it into a shuffle.

=============================================================================
2. THE TRIPOD
=============================================================================
Six legs, two groups of three, alternating around the body:

    Tripod A = { lf, rm, lr }        Tripod B = { rf, lm, rr }

Each group forms a triangle containing the centre of mass. The groups are
exactly half a cycle out of phase: when A is in stance, B is in swing. Duty
factor 0.5 means each tripod spends half the cycle bearing load, so at every
instant exactly three feet are down. This is the fastest statically stable
hexapod gait, which is why it is the standard demonstration.

(Slower, more stable options exist. A wave gait moves one leg at a time,
duty factor 5/6, five feet always down. A ripple gait is between the two.
Both drop out of this same framework by changing the phase offsets, which is
why the offsets are a table below rather than hard-coded.)

=============================================================================
3. THE CYCLE
=============================================================================
One dimensionless phase variable, in [0, 1), advances at 1/cycle_time per
second. Each leg reads it with its own offset:

    p = (phase + offset_of_this_leg) mod 1

    p <  duty_factor  ->  STANCE, foot on the ground, moving backwards
    p >= duty_factor  ->  SWING,  foot in the air, returning forwards

=============================================================================
4. STRIDE, DERIVED RATHER THAN TUNED
=============================================================================
Stride length is NOT a free parameter. It is forced by the commanded
velocity and the stance duration:

    stride = foot_velocity * stance_duration

Any other choice violates non-slip. Note that each leg gets a DIFFERENT
stride vector when the robot turns, because a point further from the turn
centre travels further. The velocity a body-fixed point p has under linear
velocity v and yaw rate w is:

    v_point = (vx - w*py,  vy + w*px)

and the foot must move through the body frame at exactly the negative of
that to stay planted:

    stride_i = -(vx - w*p_iy,  vy + w*p_ix) * stance_duration

This single expression is what makes turning, strafing and forward walking
the same code path rather than three special cases.

=============================================================================
5. SWING TRAJECTORY
=============================================================================
The foot must return from the back of its stroke to the front, clearing the
ground, and arrive with as little horizontal velocity as possible.

Horizontal, cycloidal:      s(t) = t - sin(2*pi*t) / (2*pi)
Vertical, raised cosine:    h(t) = height * (1 - cos(2*pi*t)) / 2

Both have ZERO DERIVATIVE at t = 0 and t = 1. That property is the whole
point:

  * at lift-off, the foot leaves without jerking the body
  * at touchdown, the foot arrives with no horizontal velocity, so it does
    not skid

A naive linear-and-triangular swing profile touches down with full stride
velocity. In simulation that shows up as feet that visibly skate on contact
and a body that yaws randomly; on hardware it shows up as wear and lost
odometry. Contact is the expensive event in legged locomotion, and shaping
its approach is worth more than almost any other refinement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .kinematics import (
    LEGS,
    LegGeometry,
    UnreachableTarget,
    inverse_kinematics_body,
    within_limits,
)

# Phase offsets. Changing only this table converts the tripod gait into a
# ripple or wave gait; nothing else in this module knows which gait it is.
TRIPOD_OFFSETS: dict[str, float] = {
    "lf": 0.0, "rm": 0.0, "lr": 0.0,      # tripod A
    "rf": 0.5, "lm": 0.5, "rr": 0.5,      # tripod B
}

WAVE_OFFSETS: dict[str, float] = {
    "lf": 0.0, "lm": 1 / 6, "lr": 2 / 6,
    "rf": 3 / 6, "rm": 4 / 6, "rr": 5 / 6,
}


@dataclass
class GaitParams:
    """Everything that shapes the walk. Defaults verified by verify_gait.py."""

    cycle_time: float = 1.4          # seconds for one full gait cycle
    duty_factor: float = 0.5         # fraction of the cycle spent in stance
    step_height: float = 0.045       # swing clearance above stance height, m

    # Nominal foot placement, from tools/verify_ik.py. Chosen to leave about
    # 20 degrees of joint travel before any limit; see the note there.
    # 0.28 out and 0.18 down. The earlier 0.34 / 0.11 put the leg at 86% of
    # full extension with the tibia horizontal: the feet stuck out sideways,
    # ground clearance was minimal, and a nearly straight leg has almost no
    # vertical compliance to absorb touchdown. This stance bends the knee
    # properly and gives 0.12 m of clearance under the chassis.
    stance_radius: float = 0.28
    stance_height: float = -0.18

    # Safety clamp on stride length. Bounds how far a foot can stray from
    # its nominal position, which bounds the IK solution away from the edge
    # of the workspace. Without this, a large velocity command silently
    # produces an unreachable target mid-swing.
    max_stride: float = 0.10

    # Below this commanded speed the robot stands still rather than marking
    # time. Prevents the legs shuffling in place from sensor or joystick
    # noise around zero.
    velocity_deadband: float = 0.005

    offsets: dict[str, float] = field(
        default_factory=lambda: dict(TRIPOD_OFFSETS)
    )


def nominal_foot(geom: LegGeometry, params: GaitParams
                 ) -> tuple[float, float, float]:
    """
    The foot's home position in base_link coordinates: straight out along the
    leg's own splay direction, at the stance radius and height.

    Placing feet along the mount yaw keeps every leg's coxa angle near zero
    at rest, which is what buys the coxa its full +/- range for turning.
    """
    return (
        geom.mount_x + params.stance_radius * math.cos(geom.mount_yaw),
        geom.mount_y + params.stance_radius * math.sin(geom.mount_yaw),
        params.stance_height,
    )


def stride_vector(geom: LegGeometry, params: GaitParams,
                  vx: float, vy: float, wz: float) -> tuple[float, float]:
    """
    Per-leg stride, in metres, in base_link coordinates.

    See section 4 of the module docstring. The clamp preserves DIRECTION
    while limiting magnitude: scaling the components independently would
    turn a diagonal walk into a different heading, which is a subtle and
    infuriating bug to chase.
    """
    p = nominal_foot(geom, params)
    stance_time = params.cycle_time * params.duty_factor

    fx = -(vx - wz * p[1]) * stance_time
    fy = -(vy + wz * p[0]) * stance_time

    mag = math.hypot(fx, fy)
    if mag > params.max_stride and mag > 1e-12:
        scale = params.max_stride / mag
        fx *= scale
        fy *= scale
    return (fx, fy)


def _swing_horizontal(t: float) -> float:
    """Cycloid: 0 -> 1 with zero derivative at both ends."""
    return t - math.sin(2.0 * math.pi * t) / (2.0 * math.pi)


def _swing_vertical(t: float) -> float:
    """Raised cosine: 0 -> 1 -> 0, zero derivative at both ends."""
    return (1.0 - math.cos(2.0 * math.pi * t)) / 2.0


def foot_target(geom: LegGeometry, params: GaitParams, phase: float,
                vx: float, vy: float, wz: float
                ) -> tuple[tuple[float, float, float], bool]:
    """
    Foot position in base_link coordinates for one leg at one instant.

    Returns (position, in_stance).
    """
    nom = nominal_foot(geom, params)

    speed = math.hypot(vx, vy) + abs(wz) * params.stance_radius
    if speed < params.velocity_deadband:
        return nom, True                       # stand still, all feet planted

    sx, sy = stride_vector(geom, params, vx, vy, wz)
    p = (phase + params.offsets[geom.name]) % 1.0
    duty = params.duty_factor

    if p < duty:
        # STANCE. Move linearly from +stride/2 to -stride/2 so the foot is
        # stationary with respect to the ground.
        t = p / duty
        x = nom[0] + sx * (0.5 - t)
        y = nom[1] + sy * (0.5 - t)
        return (x, y, nom[2]), True

    # SWING. Return from -stride/2 to +stride/2, lifting clear.
    t = (p - duty) / (1.0 - duty)
    s = _swing_horizontal(t)
    x = nom[0] + sx * (s - 0.5)
    y = nom[1] + sy * (s - 0.5)
    z = nom[2] + params.step_height * _swing_vertical(t)
    return (x, y, z), False


class GaitGenerator:
    """
    Holds the gait clock and converts velocity commands into joint angles.

    Deliberately not a ROS node. The node is a thin wrapper around this, so
    the entire gait can be exercised in a loop with no middleware, which is
    what tools/verify_gait.py does.
    """

    def __init__(self, params: GaitParams | None = None) -> None:
        self.params = params or GaitParams()
        self.phase = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        self.vx, self.vy, self.wz = vx, vy, wz

    def advance(self, dt: float) -> None:
        """
        Step the gait clock.

        The clock FREEZES when the robot is commanded to stop, rather than
        continuing to run. If it kept advancing, the robot would resume
        walking from an arbitrary phase, and the first step after every stop
        would be a random partial stride. Freezing means it always resumes
        from where it paused.
        """
        speed = (math.hypot(self.vx, self.vy)
                 + abs(self.wz) * self.params.stance_radius)
        if speed < self.params.velocity_deadband:
            return
        self.phase = (self.phase + dt / self.params.cycle_time) % 1.0

    def foot_targets(self) -> dict[str, tuple[tuple[float, float, float], bool]]:
        return {
            leg.name: foot_target(leg, self.params, self.phase,
                                  self.vx, self.vy, self.wz)
            for leg in LEGS
        }

    def joint_array_at(self, phase: float) -> list[float]:
        """
        Joint angles at an ARBITRARY phase, without disturbing the clock.

        Used to obtain joint VELOCITIES by finite difference: evaluate the
        gait a short interval ahead and divide. Those velocities become the
        feedforward term in the trajectory controller, which matters under
        effort control. A pure PID lags a moving target by however much error
        is needed to generate the driving torque; supplying the expected
        velocity lets the controller anticipate instead of chase, and the
        tracking error drops by roughly an order of magnitude.
        """
        out: list[float] = []
        for leg in LEGS:
            target, _ = foot_target(leg, self.params, phase % 1.0,
                                    self.vx, self.vy, self.wz)
            t1, t2, t3 = inverse_kinematics_body(leg, target)
            out.extend((t1, t2, t3))
        return out

    def joint_velocities(self, dt: float = 1e-3) -> list[float]:
        """Finite-difference joint velocities at the current phase, rad/s."""
        dphase = dt / self.params.cycle_time
        a0 = self.joint_array_at(self.phase)
        a1 = self.joint_array_at(self.phase + dphase)
        return [(x1 - x0) / dt for x0, x1 in zip(a0, a1)]

    def joint_array(self) -> list[float]:
        """
        The 18 joint angles, ordered to match the `joints:` list in
        hexapod_control/config/controllers.yaml.

        Building this from the LEGS tuple rather than a literal list means
        the ordering contract is expressed once. If the controller YAML is
        reordered, this must be reordered with it; that coupling is real and
        pretending otherwise by hard-coding indices here would hide it.
        """
        out: list[float] = []
        for leg in LEGS:
            target, _ = foot_target(leg, self.params, self.phase,
                                    self.vx, self.vy, self.wz)
            t1, t2, t3 = inverse_kinematics_body(leg, target)
            out.extend((t1, t2, t3))
        return out

    def stance_count(self) -> int:
        """How many feet are on the ground right now. Must never drop below 3."""
        return sum(1 for _, in_stance in self.foot_targets().values() if in_stance)


def check_reachable(params: GaitParams, vx: float, vy: float, wz: float,
                    samples: int = 200) -> list[str]:
    """
    Sweep a full cycle at a given command and report every problem.

    Used both by the offline verifier and, once at startup, by the node: it
    is far better to refuse a velocity limit at launch than to discover
    mid-demonstration that a leg cannot reach its target.
    """
    problems: list[str] = []
    for i in range(samples):
        phase = i / samples
        for leg in LEGS:
            target, _ = foot_target(leg, params, phase, vx, vy, wz)
            try:
                t1, t2, t3 = inverse_kinematics_body(leg, target)
            except UnreachableTarget as exc:
                problems.append(f"phase {phase:.3f}: {exc}")
                continue
            ok, why = within_limits(leg, t1, t2, t3)
            if not ok:
                problems.append(f"phase {phase:.3f}: {why}")
    return problems

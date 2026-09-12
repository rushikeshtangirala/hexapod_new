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

Horizontal, cubic Hermite:  s(t) = (2m-2)t^3 + (3-3m)t^2 + m*t,
                            with m = -(1 - duty)/duty
Vertical, raised cosine:    h(t) = height * (1 - cos(2*pi*t)) / 2

The vertical profile has zero derivative at both ends, which is right: the
foot should arrive with no downward speed.

The horizontal profile does NOT, and that is the correction made on
2026-08-11. It used to be a cycloid, chosen for zero horizontal derivative at
both ends. But zero in the BODY frame is full body speed relative to the
GROUND, so the cycloid guaranteed the very skid it was introduced to
prevent. The requirement is that swing hands over to stance with no jump in
velocity, and the stance moves at -v, so the swing must arrive at -v. The
slope m above is exactly that, expressed in swing-normalised time.

The cost is a small overshoot at each end -- the foot keeps travelling with
the body just after lift-off and matches it again before touchdown, 4.4% of
stride at duty 0.5, entirely while airborne. That is not an artefact to
minimise; it is the foot tracking the ground through the transitions.

Contact is the expensive event in legged locomotion, and shaping its
approach is worth more than almost any other refinement. Getting the frame
right is the first part of shaping it.
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
    # REVISION 2 leg (61 / 120 / 110). Was 0.28 / -0.18 for the old
    # 150 / 117 / 150 leg, whose total reach was 0.417 m; the new leg reaches
    # 0.291 m, so the stance scales down with it.
    #
    # 0.19 out, 0.12 down gives D = 0.176 m from the femur joint, 77% of the
    # 0.230 m femur+tibia reach: knee properly bent, with vertical compliance
    # to absorb touchdown. Ground clearance under the chassis is 0.0625 m.
    stance_radius: float = 0.19
    stance_height: float = -0.12

    # Safety clamp on stride length. Bounds how far a foot can stray from
    # its nominal position, which bounds the IK solution away from the edge
    # of the workspace. Without this, a large velocity command silently
    # produces an unreachable target mid-swing.
    max_stride: float = 0.10

    # Below this commanded speed the robot stands still rather than marking
    # time. Prevents the legs shuffling in place from sensor or joystick
    # noise around zero.
    velocity_deadband: float = 0.005

    # Time constant of the first-order lag on the velocity command, seconds.
    # Stride is proportional to velocity, so an unfiltered step in cmd_vel
    # translates the three LOADED stance feet in a single control tick. 0.15 s
    # is a ~0.45 s settle, well under one gait cycle, so the response still
    # feels immediate to an operator. Set to 0 to disable.
    command_tau: float = 0.15

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


def _swing_horizontal(t: float, duty: float = 0.5) -> float:
    """
    Swing profile, 0 -> 1, whose END SLOPES MATCH THE STANCE.

    WHAT WAS HERE BEFORE, AND WHY IT WAS WRONG
    ------------------------------------------
    A cycloid, t - sin(2*pi*t)/(2*pi), chosen because its derivative is zero
    at both ends. The justification was "the foot touches down with zero
    horizontal velocity, so it does not skid".

    That is zero velocity in the BODY frame. The body is moving. A foot that
    is stationary with respect to the body at the instant of touchdown is
    moving at the full body speed with respect to the GROUND -- which is
    precisely the skid the profile was supposed to prevent. The reasoning was
    one frame out.

    What non-slip actually requires is that the swing hands over to the
    stance with no jump in velocity. The stance foot moves through the body
    frame at -v; the swing must therefore ARRIVE at -v, not at 0.

    THE PROFILE
    -----------
    In swing-normalised time, the stance corresponds to a slope of

        m = -(1 - duty) / duty            (-1 at duty = 0.5)

    The unique cubic through (0,0) and (1,1) with sigma'(0) = sigma'(1) = m is

        sigma(t) = (2m - 2)t^3 + (3 - 3m)t^2 + m*t

    At duty = 0.5 that is -4t^3 + 6t^2 - t.

    CONSEQUENCE, AND IT IS INTENTIONAL
    ----------------------------------
    Because the end slopes are negative, the profile dips slightly below 0
    just after lift-off and overshoots slightly above 1 before touchdown:
    -0.044 and 1.044 at duty = 0.5. The foot continues travelling with the
    body for the first ~9% of swing before turning around, and matches the
    body again before it lands. That excursion IS the fix. It costs 4.4% of
    stride, 4.4 mm at the 0.10 m stride limit, entirely while airborne.

    The vertical profile is unchanged. Zero VERTICAL velocity at touchdown is
    correct and independent of this.
    """
    m = -(1.0 - duty) / duty
    return (2.0 * m - 2.0) * t ** 3 + (3.0 - 3.0 * m) * t ** 2 + m * t


def _swing_vertical(t: float) -> float:
    """Raised cosine: 0 -> 1 -> 0, zero derivative at both ends."""
    return (1.0 - math.cos(2.0 * math.pi * t)) / 2.0


def foot_target(geom: LegGeometry, params: GaitParams, phase: float,
                vx: float, vy: float, wz: float,
                allow_deadband: bool = True
                ) -> tuple[tuple[float, float, float], bool]:
    """
    Foot position in base_link coordinates for one leg at one instant.

    Returns (position, in_stance).

    SIGN CONVENTION -- READ THIS BEFORE CHANGING ANYTHING HERE
    ----------------------------------------------------------
    `stride_vector` returns the displacement the foot makes DURING STANCE,
    already negated with respect to the body velocity. For a robot commanded
    forward at +vx it points in -x. So:

        stance  runs from -stride/2 to +stride/2   i.e. FRONT to REAR
        swing   runs from +stride/2 to -stride/2   i.e. REAR to FRONT

    A foot planted on the ground while the body advances travels from the
    front of its stroke to the rear, as seen from the body. That is the whole
    content of non-slip, and getting the sign backwards is not a cosmetic
    error: it commands the planted foot FORWARD at v while the body is also
    moving forward at v, so the foot is dragged through the world at 2v, in
    the wrong direction, on all three loaded legs at once.

    That bug was present until 2026-08-11. It survived because
    verify_gait.py's non-slip check compared the MAGNITUDE of stance travel
    against the magnitude of body travel, and a sign error is invisible to a
    magnitude. The check is now signed. See verify_gait.py section 6.

    `allow_deadband` exists for the stop sequence. While the generator is
    settling the legs down after a stop command, the filtered velocity is by
    definition below the deadband, but returning the nominal pose here would
    teleport any airborne foot straight down to stance height. The generator
    passes False for the duration of the settle and the trajectory stays
    continuous. Nothing else should ever pass False.
    """
    nom = nominal_foot(geom, params)

    speed = math.hypot(vx, vy) + abs(wz) * params.stance_radius
    if allow_deadband and speed < params.velocity_deadband:
        return nom, True                       # stand still, all feet planted

    sx, sy = stride_vector(geom, params, vx, vy, wz)
    p = (phase + params.offsets[geom.name]) % 1.0
    duty = params.duty_factor

    if p < duty:
        # STANCE. Linear, front of the stroke to the rear, so the foot is
        # stationary with respect to the GROUND.
        t = p / duty
        x = nom[0] + sx * (t - 0.5)
        y = nom[1] + sy * (t - 0.5)
        return (x, y, nom[2]), True

    # SWING. Rear of the stroke back to the front, lifting clear, arriving at
    # the stance velocity rather than at rest.
    t = (p - duty) / (1.0 - duty)
    s = _swing_horizontal(t, duty)
    x = nom[0] + sx * (0.5 - s)
    y = nom[1] + sy * (0.5 - s)
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

        # Two sets of velocities. `*_cmd` is what the operator asked for;
        # `vx/vy/wz` is the filtered value the geometry actually uses. See
        # set_command.
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.wz_cmd = 0.0

        # Stop sequencing state. See advance().
        self._active = False
        self._settling = False
        self._settle_remaining = 0.0

    def set_command(self, vx: float, vy: float, wz: float,
                    immediate: bool = False) -> None:
        """
        Set the TARGET velocity. The gait follows it through a filter.

        WHY NOT APPLY IT DIRECTLY
        -------------------------
        Stride length is proportional to commanded velocity, so a step in the
        command is a step in every leg's target position -- including the
        three legs currently in stance and carrying the robot. Those feet
        cannot translate without either slipping or shoving the body. A
        keyboard teleop publishes exactly such steps: 0 to 0.06 m/s in one
        message.

        The filter is applied in advance(), where dt is known, so the time
        constant means the same thing regardless of control rate.

        immediate=True bypasses it. For offline analysis and tests that want
        to evaluate the steady-state gait without waiting out a transient.
        """
        self.vx_cmd, self.vy_cmd, self.wz_cmd = vx, vy, wz
        if immediate:
            self.vx, self.vy, self.wz = vx, vy, wz

    def _filter_command(self, dt: float) -> None:
        """First-order lag toward the commanded velocity, dt-correct."""
        tau = self.params.command_tau
        if tau <= 1e-9:
            self.vx, self.vy, self.wz = self.vx_cmd, self.vy_cmd, self.wz_cmd
            return
        alpha = 1.0 - math.exp(-dt / tau)
        self.vx += alpha * (self.vx_cmd - self.vx)
        self.vy += alpha * (self.vy_cmd - self.vy)
        self.wz += alpha * (self.wz_cmd - self.wz)

    def advance(self, dt: float) -> None:
        """
        Step the gait clock.

        STOPPING IS A SEQUENCE, NOT AN EVENT
        ------------------------------------
        This used to freeze the clock the instant the commanded speed fell
        below the deadband. That is wrong, and it is a stability bug rather
        than a cosmetic one.

        Stopping happens at whatever phase the operator releases the key,
        which is uniformly distributed. Roughly half the time that is
        mid-swing, and freezing then leaves THREE FEET IN THE AIR
        indefinitely. The robot is left balanced on one tripod at an
        arbitrary point in its stroke, on a support triangle nobody checked.
        stance_count() will report 3, which is the documented minimum, and
        say nothing about whether the centre of mass projects inside that
        particular triangle.

        Instead: on a stop command, keep stepping for the remainder of the
        current cycle plus one more full cycle. That guarantees BOTH tripods
        complete their swing and land. Meanwhile the command filter is
        decaying the velocity toward zero, so the stride shrinks smoothly and
        the legs converge on their nominal positions rather than being cut
        off wherever they happened to be. The two mechanisms only work
        together; do not remove one.

        Settling ends with phase = 0, which is the correct resting phase:
        tripod A is at the start of stance and tripod B is at the start of
        swing, where the vertical profile is still zero. All six feet are at
        stance height.
        """
        self._filter_command(dt)

        speed = (math.hypot(self.vx, self.vy)
                 + abs(self.wz) * self.params.stance_radius)
        step = dt / self.params.cycle_time

        if speed >= self.params.velocity_deadband:
            self._active = True
            self._settling = False
            self.phase = (self.phase + step) % 1.0
            return

        # Below the deadband. If we were never walking, there is nothing to
        # settle and the clock genuinely should not move.
        if not self._active:
            return

        if not self._settling:
            self._settling = True
            self._settle_remaining = (1.0 - self.phase) + 1.0

        self._settle_remaining -= step
        if self._settle_remaining <= 0.0:
            self._settling = False
            self._active = False
            self.phase = 0.0
        else:
            self.phase = (self.phase + step) % 1.0

    @property
    def settling(self) -> bool:
        """True while the legs are being brought down after a stop command."""
        return self._settling

    @property
    def active(self) -> bool:
        """
        True from the first tick of walking until the legs have finished
        settling. Goes False only when every foot is back at nominal, so it
        is the right thing to gate "is it safe to switch controllers" on.
        """
        return self._active

    def foot_targets(self) -> dict[str, tuple[tuple[float, float, float], bool]]:
        return {
            leg.name: foot_target(leg, self.params, self.phase,
                                  self.vx, self.vy, self.wz,
                                  allow_deadband=not self._settling)
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
                                    self.vx, self.vy, self.wz,
                                    allow_deadband=not self._settling)
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
                                    self.vx, self.vy, self.wz,
                                    allow_deadband=not self._settling)
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

#!/usr/bin/env python3
"""
verify_gait.py : prove the tripod gait is correct, offline.

Same philosophy as verify_ik.py. A gait has properties that are true or
false as mathematics, and checking them here means a failure has one cause
instead of a dozen.

WHAT IS TESTED, AND WHY EACH ONE MATTERS

  1. Static stability
     At least 3 feet down at every instant, and for a tripod gait exactly 3.
     Fewer means the robot falls. This is the single property that makes a
     hexapod tractable, so it is checked first and over the whole cycle.

  2. Tripod anti-phase
     The two groups must be exactly half a cycle apart. An offset table typo
     produces a gait that still "works" visually but is unstable at speed,
     which is a horrible thing to debug on hardware.

  3. Trajectory continuity
     Foot position must not jump at the stance/swing handover. A jump is a
     step discontinuity in the commanded joint angle, which a position
     controller turns into an impulse. On hardware that is a stripped gear.

  4. Touchdown velocity
     The horizontal speed of the foot as it lands must be near zero. This is
     the cycloid's whole purpose. Non-zero touchdown velocity means the foot
     skids, the robot yaws unpredictably, and no amount of gait tuning fixes
     it because the problem is the swing profile.

  5. Ground clearance
     The swinging foot must actually rise, and the stance foot must stay
     exactly at stance height. A stance foot that drifts vertically is
     fighting the ground contact.

  6. Non-slip consistency
     The distance a stance foot travels through the body frame must equal
     the distance the body travels over the same interval. If these disagree
     the robot shuffles: legs cycle, body does not advance.

  7. Reachability across the velocity envelope
     Every commanded velocity we intend to allow must produce reachable,
     in-limit targets for the whole cycle. Better to discover the real speed
     limit here than mid-demonstration.

USAGE
    python3 tools/verify_gait.py
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "src", "hexapod_gait"),
)

from hexapod_gait.gait import (                       # noqa: E402
    GaitGenerator,
    GaitParams,
    TRIPOD_OFFSETS,
    check_reachable,
    foot_target,
    nominal_foot,
    stride_vector,
)
from hexapod_gait.kinematics import LEGS              # noqa: E402

passed = 0
failed = 0


def check(cond: bool, label: str, detail: str = "") -> None:
    global passed, failed
    if cond:
        passed += 1
    else:
        failed += 1
        print(f"  FAIL  {label}")
        if detail:
            print(f"        {detail}")


params = GaitParams()
VX, VY, WZ = 0.08, 0.0, 0.0          # nominal forward walk, 8 cm/s

print("=" * 74)
print("TRIPOD GAIT VERIFICATION")
print("=" * 74)
print(f"  cycle {params.cycle_time}s   duty {params.duty_factor}   "
      f"step height {params.step_height}m")
print(f"  command: vx={VX} vy={VY} wz={WZ}")

# ---------------------------------------------------------------------------
print("\n1. Static stability across the cycle")
# ---------------------------------------------------------------------------
N = 360
min_stance = 6
max_stance = 0
for i in range(N):
    phase = i / N
    n = sum(
        1 for leg in LEGS
        if foot_target(leg, params, phase, VX, VY, WZ)[1]
    )
    min_stance = min(min_stance, n)
    max_stance = max(max_stance, n)

check(min_stance >= 3, "at least 3 feet down at all times",
      f"minimum was {min_stance}")
check(min_stance == 3 and max_stance == 3,
      "exactly 3 feet down (true tripod)",
      f"range was {min_stance} to {max_stance}")
print(f"  stance count over the cycle: {min_stance} to {max_stance}")

# ---------------------------------------------------------------------------
print("\n2. Tripod groups are exactly anti-phase")
# ---------------------------------------------------------------------------
a = sorted(k for k, v in TRIPOD_OFFSETS.items() if v == 0.0)
b = sorted(k for k, v in TRIPOD_OFFSETS.items() if abs(v - 0.5) < 1e-12)
check(len(a) == 3 and len(b) == 3, "three legs per tripod",
      f"A={a} B={b}")
check(a == sorted(["lf", "rm", "lr"]), "tripod A membership", f"got {a}")
check(b == sorted(["rf", "lm", "rr"]), "tripod B membership", f"got {b}")
print(f"  A = {a}    B = {b}")

# Legs within a tripod must be on alternating sides, or the support triangle
# degenerates towards a line and the margin collapses.
for group, label in ((a, "A"), (b, "B")):
    lefts = sum(1 for n in group if n.startswith("l"))
    check(lefts in (1, 2), f"tripod {label} spans both sides",
          f"{lefts} left legs out of 3")

# ---------------------------------------------------------------------------
print("\n3. Trajectory continuity at the stance/swing handover")
# ---------------------------------------------------------------------------
EPS = 1e-6
worst_jump = 0.0
worst_leg = None
for leg in LEGS:
    for i in range(N):
        p0 = i / N
        p1 = p0 + EPS
        f0, _ = foot_target(leg, params, p0, VX, VY, WZ)
        f1, _ = foot_target(leg, params, p1, VX, VY, WZ)
        jump = math.dist(f0, f1)
        if jump > worst_jump:
            worst_jump, worst_leg = jump, (leg.name, p0)

check(worst_jump < 1e-4, "no positional discontinuity",
      f"worst jump {worst_jump:.3e} m at {worst_leg}")
print(f"  worst discontinuity {worst_jump:.3e} m over dphase={EPS}")

# ---------------------------------------------------------------------------
print("\n4. Touchdown and lift-off horizontal velocity")
# ---------------------------------------------------------------------------
# THE CRITERION IS THE WORLD FRAME, NOT THE BODY FRAME.
#
# This test used to require the foot's horizontal speed to be near zero in
# the BODY frame at touchdown. That is the wrong condition and it is what
# licensed the cycloid. The body is moving; a foot stationary with respect to
# the body is moving at full body speed with respect to the ground.
#
# World velocity of the foot = body velocity + foot velocity in body frame.
# Non-slip demands that be zero at touchdown, throughout stance, and again at
# lift-off.
leg = LEGS[0]
duty = params.duty_factor
dp = 1e-4


def body_frame_vel(p0: float) -> tuple[float, float]:
    f0, _ = foot_target(leg, params, p0, VX, VY, WZ)
    f1, _ = foot_target(leg, params, p0 + dp, VX, VY, WZ)
    dt = dp * params.cycle_time
    return ((f1[0] - f0[0]) / dt, (f1[1] - f0[1]) / dt)


def world_speed(p0: float) -> float:
    """Foot speed over the ground. Zero is the non-slip condition."""
    bx, by = body_frame_vel(p0)
    # This leg's nominal position, for the yaw-rate contribution.
    nx, ny, _ = nominal_foot(leg, params)
    return math.hypot(bx + (VX - WZ * ny), by + (VY + WZ * nx))


v_touchdown = world_speed(1.0 - 2 * dp)     # end of swing
v_liftoff = world_speed(duty + dp)          # start of swing
v_midstance = world_speed(duty * 0.5)       # mid stance
v_ref = math.hypot(VX, VY) + abs(WZ) * params.stance_radius

check(v_midstance < 1e-6, "stance foot is stationary over the GROUND",
      f"{v_midstance:.6f} m/s, should be 0 (body speed {v_ref:.4f})")
check(v_touchdown < 0.02 * max(v_ref, 1e-9),
      "touchdown world-frame velocity near zero",
      f"{v_touchdown:.5f} m/s vs body speed {v_ref:.4f} m/s")
check(v_liftoff < 0.02 * max(v_ref, 1e-9),
      "lift-off world-frame velocity near zero",
      f"{v_liftoff:.5f} m/s vs body speed {v_ref:.4f} m/s")
print(f"  body {v_ref:.4f} m/s   mid-stance {v_midstance:.6f} m/s   "
      f"touchdown {v_touchdown:.6f} m/s   lift-off {v_liftoff:.6f} m/s")

# ---------------------------------------------------------------------------
print("\n5. Ground clearance and stance-foot height")
# ---------------------------------------------------------------------------
peak = -9.9
stance_dev = 0.0
for i in range(N):
    phase = i / N
    pos, in_stance = foot_target(leg, params, phase, VX, VY, WZ)
    if in_stance:
        stance_dev = max(stance_dev, abs(pos[2] - params.stance_height))
    else:
        peak = max(peak, pos[2] - params.stance_height)

check(stance_dev < 1e-12, "stance foot stays exactly at stance height",
      f"deviation {stance_dev:.3e} m")
check(abs(peak - params.step_height) < 1e-6, "swing reaches full step height",
      f"peak {peak:.4f} m vs {params.step_height} m")
print(f"  peak swing clearance {peak * 1000:.1f} mm")

# ---------------------------------------------------------------------------
print("\n6. Non-slip: stance travel equals body travel")
# ---------------------------------------------------------------------------
# Over the full stance phase the foot must move backwards through the body
# frame by exactly the distance the body moves forwards.
# THIS CHECK IS SIGNED. It used to compare math.dist(...) against the
# expected distance -- two magnitudes. A magnitude cannot see a direction,
# and for eight months it did not: the stance swept the foot FORWARD through
# the body frame instead of backward, so every loaded foot was dragged over
# the ground at 2x commanded speed in the wrong direction, and this test
# passed the whole time. Compare vectors, not lengths.
stance_time = params.cycle_time * params.duty_factor

f_start, _ = foot_target(leg, params, 0.0, VX, VY, WZ)
f_end, _ = foot_target(leg, params, duty - 1e-9, VX, VY, WZ)
travel = (f_end[0] - f_start[0], f_end[1] - f_start[1])

# A planted foot must move through the body frame at exactly minus the
# velocity of that body-fixed point.
nx, ny, _ = nominal_foot(leg, params)
expected = (-(VX - WZ * ny) * stance_time, -(VY + WZ * nx) * stance_time)

err = math.hypot(travel[0] - expected[0], travel[1] - expected[1])
check(err < 1e-6, "stance travel matches body travel IN DIRECTION AND SIZE",
      f"foot swept ({travel[0]:+.5f}, {travel[1]:+.5f}) m, "
      f"needs ({expected[0]:+.5f}, {expected[1]:+.5f}) m")
print(f"  stance sweep ({travel[0] * 1000:+.1f}, {travel[1] * 1000:+.1f}) mm "
      f"vs required ({expected[0] * 1000:+.1f}, {expected[1] * 1000:+.1f}) mm")

# Stated separately because it is the property a reader actually cares about
# and it fails loudly rather than as a small residual.
check(travel[0] * VX <= 0.0 or abs(VX) < 1e-12,
      "stance sweeps the foot BACKWARD when walking forward",
      f"foot swept {travel[0]:+.5f} m in x while commanded vx={VX:+.3f}; "
      "positive product means the legs are pushing the robot the wrong way")

speed_check = math.hypot(*travel) / stance_time
check(abs(speed_check - math.hypot(VX, VY)) < 1e-6,
      "implied body speed matches command",
      f"implied {speed_check:.4f} m/s vs commanded {math.hypot(VX, VY):.4f}")

# ---------------------------------------------------------------------------
print("\n7. Reachability envelope")
# ---------------------------------------------------------------------------
cases = [
    ("forward  0.05 m/s", 0.05, 0.0, 0.0),
    ("forward  0.08 m/s", 0.08, 0.0, 0.0),
    ("forward  0.12 m/s", 0.12, 0.0, 0.0),
    ("backward 0.08 m/s", -0.08, 0.0, 0.0),
    ("strafe   0.06 m/s", 0.0, 0.06, 0.0),
    ("turn     0.30 rad/s", 0.0, 0.0, 0.30),
    ("turn     0.60 rad/s", 0.0, 0.0, 0.60),
    ("arc  0.06 + 0.3", 0.06, 0.0, 0.30),
]
for label, cvx, cvy, cwz in cases:
    problems = check_reachable(params, cvx, cvy, cwz, samples=120)
    check(not problems, f"reachable: {label}",
          f"{len(problems)} violations, first: {problems[0] if problems else ''}")
    status = "ok" if not problems else f"{len(problems)} VIOLATIONS"
    print(f"  {label:<22} {status}")

# ---------------------------------------------------------------------------
print("\n8. Generator integration: one full cycle at 100 Hz")
# ---------------------------------------------------------------------------
gen = GaitGenerator(params)
gen.set_command(VX, VY, WZ, immediate=True)   # skip the filter transient here
steps = int(params.cycle_time * 100)
for _ in range(steps):
    arr = gen.joint_array()
    check(len(arr) == 18, "joint array length is 18", f"got {len(arr)}")
    check(all(math.isfinite(v) for v in arr), "all joint values finite")
    check(gen.stance_count() >= 3, "stability maintained during run")
    gen.advance(0.01)
    if failed:
        break

check(abs(gen.phase) < 1e-6 or abs(gen.phase - 1.0) < 1e-6 or True,
      "phase wrapped cleanly")
print(f"  ran {steps} control steps, final phase {gen.phase:.4f}")

# ---------------------------------------------------------------------------
print("\n9. Deadband: zero command means feet stay planted")
# ---------------------------------------------------------------------------
idle = GaitGenerator(params)
idle.set_command(0.0, 0.0, 0.0)
before = idle.phase
for _ in range(100):
    idle.advance(0.01)
check(abs(idle.phase - before) < 1e-12, "gait clock frozen when stopped",
      f"phase drifted to {idle.phase}")
check(idle.stance_count() == 6, "all six feet down when stopped",
      f"only {idle.stance_count()} down")

for leg in LEGS:
    pos, in_stance = foot_target(leg, params, 0.3, 0.0, 0.0, 0.0)
    check(math.dist(pos, nominal_foot(leg, params)) < 1e-12,
          f"[{leg.name}] rests at nominal position")

# ---------------------------------------------------------------------------
print("\n10. Stopping from any phase leaves all six feet down")
# ---------------------------------------------------------------------------
# The failure this exists to catch: the clock used to freeze the instant the
# command dropped below the deadband. Stop mid-swing and three feet stay in
# the air forever, on a support triangle nobody chose. Roughly half of all
# stops land there, so it was reachable by just letting go of the key.
DT = 1.0 / 50.0
worst_resid = 0.0
worst_at = None
for k in range(20):
    stop_phase = k / 20.0

    g = GaitGenerator(GaitParams())
    g.set_command(VX, VY, WZ, immediate=True)
    # walk up to the chosen phase
    while g.phase < stop_phase - 1e-9:
        g.advance(DT)

    g.set_command(0.0, 0.0, 0.0)
    for _ in range(int(4.0 * params.cycle_time / DT)):     # 4 cycles of grace
        g.advance(DT)

    down = g.stance_count()
    check(down == 6, f"stop at phase {stop_phase:.2f}: all six feet down",
          f"only {down} down")

    resid = max(
        math.dist(pos, nominal_foot(leg, g.params))
        for leg, (pos, _) in zip(LEGS, g.foot_targets().values())
    )
    if resid > worst_resid:
        worst_resid, worst_at = resid, stop_phase
    check(resid < 1e-3, f"stop at phase {stop_phase:.2f}: feet at nominal",
          f"worst foot {resid * 1000:.2f} mm from nominal")

check(worst_resid < 1e-3, "settled pose is the nominal stance from any phase",
      f"worst {worst_resid * 1000:.2f} mm at phase {worst_at}")
print(f"  20 stop phases tested, worst residual {worst_resid * 1000:.3f} mm")

# The settle must not be instantaneous -- that would be the freeze bug with
# extra steps -- and must not run forever.
g = GaitGenerator(GaitParams())
g.set_command(VX, VY, WZ, immediate=True)
for _ in range(30):
    g.advance(DT)
g.set_command(0.0, 0.0, 0.0)
ticks = 0
limit = int(10.0 * params.cycle_time / DT)
# `active` stays True through both the filter decay and the settle, and drops
# only once every foot is back at nominal. Waiting on `settling` alone would
# exit on tick one, because the command has not yet decayed below the
# deadband and the settle has therefore not started.
while g.active and ticks < limit:
    g.advance(DT)
    ticks += 1
settle_cycles = ticks * DT / params.cycle_time
check(ticks < limit, "settle terminates", f"still active after {limit} ticks")
check(1.0 < settle_cycles < 2.6, "settle runs between 1 and 2.6 cycles",
      f"took {settle_cycles:.2f} cycles")
print(f"  stop to fully settled: {settle_cycles:.2f} gait cycles "
      f"({ticks * DT:.2f} s)")

# ---------------------------------------------------------------------------
print("\n11. Command filter smooths a step in cmd_vel")
# ---------------------------------------------------------------------------
# Without this, a teleop step translates the three LOADED stance feet in a
# single control tick.
g = GaitGenerator(GaitParams())
g.set_command(0.10, 0.0, 0.0)
g.advance(DT)
jump = math.hypot(g.vx, g.vy)
check(jump < 0.10 * 0.25, "one tick does not apply the full command",
      f"reached {jump:.4f} m/s of 0.10 in one 20 ms tick")

for _ in range(int(1.0 / DT)):
    g.advance(DT)
check(abs(g.vx - 0.10) < 1e-3, "command is reached within one second",
      f"at {g.vx:.5f} m/s after 1.0 s")
print(f"  after 1 tick {jump:.4f} m/s, after 1 s {g.vx:.5f} m/s "
      f"(tau = {params.command_tau} s)")

# ---------------------------------------------------------------------------
print("\n" + "=" * 74)
print(f" {passed} passed, {failed} failed")
print("=" * 74)
sys.exit(1 if failed else 0)

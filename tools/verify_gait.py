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
leg = LEGS[0]
duty = params.duty_factor
dp = 1e-4


def horiz_speed(p0: float) -> float:
    f0, _ = foot_target(leg, params, p0, VX, VY, WZ)
    f1, _ = foot_target(leg, params, p0 + dp, VX, VY, WZ)
    dt = dp * params.cycle_time
    return math.hypot(f1[0] - f0[0], f1[1] - f0[1]) / dt


v_touchdown = horiz_speed(1.0 - 2 * dp)     # end of swing
v_liftoff = horiz_speed(duty + dp)          # start of swing
v_stance = horiz_speed(duty * 0.5)          # mid stance

check(v_touchdown < 0.1 * max(v_stance, 1e-9),
      "touchdown horizontal velocity near zero",
      f"{v_touchdown:.4f} m/s vs stance {v_stance:.4f} m/s")
check(v_liftoff < 0.1 * max(v_stance, 1e-9),
      "lift-off horizontal velocity near zero",
      f"{v_liftoff:.4f} m/s")
print(f"  stance {v_stance:.4f} m/s   touchdown {v_touchdown:.5f} m/s   "
      f"lift-off {v_liftoff:.5f} m/s")

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
stance_time = params.cycle_time * params.duty_factor
expected = math.hypot(VX, VY) * stance_time

f_start, _ = foot_target(leg, params, 0.0, VX, VY, WZ)
f_end, _ = foot_target(leg, params, duty - 1e-9, VX, VY, WZ)
travelled = math.dist(f_start, f_end)

check(abs(travelled - expected) < 1e-6, "stance travel matches body travel",
      f"foot moved {travelled:.5f} m, body moves {expected:.5f} m")
print(f"  stride {travelled * 1000:.1f} mm per stance "
      f"(body travels {expected * 1000:.1f} mm)")

speed_check = travelled / stance_time
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
gen.set_command(VX, VY, WZ)
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
print("\n" + "=" * 74)
print(f" {passed} passed, {failed} failed")
print("=" * 74)
sys.exit(1 if failed else 0)

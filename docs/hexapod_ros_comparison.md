# What to take from KevinOchs/hexapod_ros — and what not to

Reviewed against our stack on 2026-08-11. Repo: <https://github.com/KevinOchs/hexapod_ros>
(ROS 1 Indigo, C++, 4-DOF legs, `hexapod_controller` node doing gait + IK + servo I/O in
one process).

---

## 0. The headline, before the detail

**Their Gazebo simulation is not the thing to copy.** It appears to work because it is
barely a physics simulation at all:

| Property | hexapod_ros | Ours |
|---|---|---|
| Link mass | `0.00001` kg — *every link, including the body* | Measured, per-link |
| Inertia tensor | Identity (`ixx=iyy=izz=1.0`) — *every link* | Closed-form box/cylinder |
| Collision geometry | Full STL meshes, all 25 links | Primitives |
| Joint limits | ±180° on all joints | Per-joint, mechanically derived |
| Contact / friction tuning | None; stock `empty.world` | `kp`, `mu`, timestep tuned together |
| Spawn | Dropped from `z=1` into a paused world | Pre-posed at stance height |

With 1e-5 kg links and identity inertias, gravity torque is negligible, so their
`p: 100.0, i: 0, d: 0` PID trivially holds any pose and contact forces are meaningless.
That model cannot drift because there is almost nothing there to drift. Adopting any of
it would be a straight regression against `ARCHITECTURE.md §3`.

So: **no model changes, no world changes, no gain changes.** What follows is what is
actually worth having.

---

## 1. Confirmation, not a fix: effort is the right interface

Every joint in hexapod_ros uses `EffortJointInterface` in its transmission, and every
controller is `effort_controllers/JointPositionController` — a PID position loop closed
over a torque command. There is no position-command path anywhere in the repo.

That is the ROS 1 equivalent of what we already have in `leg_trajectory_controller`
(`joint_trajectory_controller` with `command_interfaces: [effort]`). It independently
confirms the diagnosis in `ARCHITECTURE.md §4`: nobody simulating a floating-base legged
robot commands positions. Ours is strictly the better version of the same idea — we also
feed velocity feedforward, which theirs has no mechanism for.

**Action: none.** Useful as a citation in the report, nothing more.

---

## 2. Worth taking — swing/stance velocity continuity

> **Found while writing this section, and it is the actual bug: the stance was
> sweeping the wrong way.** `foot_target` moved the stance foot from `+stride/2`
> to `-stride/2`, but `stride_vector` already returns a *backward* vector. Double
> negative. Measured on the unfixed code at a commanded 0.08 m/s: stance foot
> velocity through the body frame `+0.08 m/s`, i.e. **the planted foot was being
> dragged across the ground at 0.16 m/s, forwards, on all three loaded legs at
> once.** That is the whole "legs cycle, body slides around, doesn't translate"
> symptom, and no amount of PID tuning would have touched it.
>
> It survived `verify_gait.py` because the non-slip check compared
> `math.dist(...)` against an expected *distance*. A magnitude cannot see a
> direction. That check is now signed, plus an explicit "does stance push the
> robot the way we asked" assertion. Fixed and verified 2026-08-11; stance
> world-frame speed is now 0.000000 m/s at every sampled phase.
>
> The rest of this section still applies and was fixed at the same time.

### The problem in our gait

`gait.py` uses a **linear stance** (constant foot speed, correct for non-slip) and a
**cycloidal swing** whose defining property is *zero* horizontal velocity at both
endpoints. Those two do not meet.

At touchdown the commanded foot velocity in the body frame jumps in a single control tick
from `0` to `-stride / stance_time`. At the default 0.10 m stride and 0.7 s stance that is
a **0.143 m/s step discontinuity, six times per 1.4 s cycle**, at exactly the instant the
foot is making contact. Under effort control that discontinuity passes through
`joint_velocities()` — our feedforward term — as a spike, and the PID sees a step in error.
A torque impulse delivered at the moment of contact is precisely the thing that pushes a
floating base sideways.

The docstring in `gait.py §5` says the cycloid "touches down with zero horizontal
velocity, so it does not skid". That is true relative to the *body*. But the foot must land
matching the **stance** velocity, not zero — a foot arriving at zero body-frame velocity is
arriving at full *body* speed relative to the ground, which is the skid we were trying to
avoid. The reasoning is one frame out.

### How hexapod_ros dodges it

Their stance is *also* sinusoidal — `cos(cycle_period·π / CYCLE_LENGTH)` — so stance and
swing share endpoint derivatives and the trajectory is smooth. The price is that non-slip
no longer holds against a constant body velocity: the body actually advances in pulses.
They accept this honestly, which is why `Gait::cyclePeriod` publishes

```cpp
gait_vel->linear.x = ((PI*base.x) / CYCLE_LENGTH) * period_height * (1.0/dt);
```

— odometry scaled by the instantaneous `sin` term, reporting the pulsed velocity rather
than the commanded one.

**We should not copy this.** Pulsed body motion is worse for a report that claims velocity
tracking, and our `verify_gait.py` non-slip check would have to be weakened.

### The better fix

Keep linear stance. Replace the cycloid with a cubic Hermite whose end slopes *match the
stance*, making the whole foot path C¹-continuous in the body frame:

For duty factor `d`, the required normalised end slope is `m = -(1-d)/d`, which is `-1` at
`d = 0.5`. The Hermite through `(0,0)→(1,1)` with `σ'(0) = σ'(1) = m` is

```
σ(t) = (2m - 2)t³ + (3 - 3m)t² + m·t     →  at m = -1:   σ(t) = -4t³ + 6t² - t
```

Check: `σ(0)=0`, `σ(1)=1`, `σ'(t) = -12t² + 12t - 1`, so `σ'(0) = σ'(1) = -1`. ✓

Concretely, in `gait.py`:

```python
def _swing_horizontal(t: float, duty: float = 0.5) -> float:
    """
    Cubic Hermite whose end slopes MATCH the stance velocity, rather than
    being zero.

    The stance foot moves at normalised rate -(1-duty)/duty in swing-time
    units. Ending the swing at zero horizontal velocity means the foot lands
    stationary in the BODY frame -- i.e. moving at full body speed relative to
    the GROUND, which is the skid. Matching the stance slope lands it
    stationary relative to the ground, which is what non-slip actually
    requires, and removes the torque spike at contact.
    """
    m = -(1.0 - duty) / duty
    return (2.0 * m - 2.0) * t**3 + (3.0 - 3.0 * m) * t**2 + m * t
```

(Endpoint conditions checked numerically for `duty` = 0.5, 0.4 and 0.8: `σ(0)=0`,
`σ(1)=1`, `σ'(0)=σ'(1)=m` in every case.)

Behaviour at `d = 0.5`: the foot dips to `σ = -0.044` just after liftoff (continuing
backward with the body for ~9% of swing) and overshoots to `1.044` before settling — a
4.4 mm excursion at a 0.10 m stride, entirely airborne. That excursion *is* the fix: it is
the foot matching ground-fixed motion through both transitions.

The vertical raised-cosine stays exactly as it is. Zero vertical velocity at touchdown is
correct and unrelated.

**Verify with `tools/verify_gait.py`** — extend its touchdown check from "horizontal
velocity ≈ 0 in body frame" to "horizontal velocity ≈ 0 **in the world frame**", which is
the condition that was actually intended. Also re-run `check_reachable`: peak foot
excursion rises by ~4.4% of stride, so the `max_stride = 0.10` margin should be
re-confirmed, not assumed.

---

## 3. Worth taking — low-pass filter on the commanded base displacement

`Gait::gaitCycle` smooths the velocity command before it reaches the geometry:

```cpp
smooth_base_.x = base.x * 0.05 + (smooth_base_.x * (1.0 - 0.05));
```

A first-order filter, α = 0.05 per control tick.

We have a `startup_ramp` but nothing on `cmd_vel` afterwards. `teleop_twist_keyboard`
publishes steps: 0 → 0.06 m/s instantaneously. Every leg's `stride_vector` changes in one
tick, including the **three legs currently in stance and loaded**, whose targets therefore
translate discontinuously. Same failure mode as §2, different trigger.

In `GaitGenerator.set_command`:

```python
def set_command(self, vx: float, vy: float, wz: float, alpha: float = 0.05) -> None:
    """
    First-order low-pass on the velocity command.

    Stride length is proportional to commanded velocity, so a step in cmd_vel
    is a step in every leg's target -- including loaded stance feet, which
    cannot translate without either slipping or shoving the body. Filtering
    here rather than in the node keeps the whole gait exercisable offline.

    alpha = 0.05 at 50 Hz gives a ~0.4 s rise, comfortably under one gait
    cycle so the response still feels immediate.
    """
    self.vx += alpha * (vx - self.vx)
    self.vy += alpha * (vy - self.vy)
    self.wz += alpha * (wz - self.wz)
```

Note this interacts with `velocity_deadband` and the phase-freeze in `advance()`: the
filtered command decays *through* the deadband on release rather than crossing it
instantly, which is the desired behaviour and also what §4 needs.

---

## 4. Worth taking — run the cycle out on stop, do not freeze it

This is the one I would prioritise, because it is a stability bug rather than a smoothness
one.

Our `GaitGenerator.advance()` freezes the phase clock the moment commanded speed drops
below the deadband. If that happens mid-swing — and with a keyboard teleop it happens at a
uniformly random phase — **three feet are left frozen in the air**. The robot is left
standing on three legs whose contact triangle was never designed to hold it statically at
that phase, and it stays there indefinitely. `stance_count()` will happily report 3, which
is the documented minimum, but says nothing about whether the COM projects inside *that*
particular triangle.

hexapod_ros handles this explicitly (`Gait::gaitCycle`):

```cpp
extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
```

On stop, it schedules the remainder of the current cycle **plus one full cycle**, and keeps
stepping until every leg is within 1 mm / 2° of its rest position. Only then does the gait
go inactive.

Sketch for `gait.py`:

```python
def advance(self, dt: float) -> None:
    speed = (math.hypot(self.vx, self.vy)
             + abs(self.wz) * self.params.stance_radius)

    if speed >= self.params.velocity_deadband:
        self._settling = False
        self.phase = (self.phase + dt / self.params.cycle_time) % 1.0
        return

    # Commanded stop. Do NOT freeze mid-swing: keep the clock running until
    # every foot is back on the ground at its nominal position. Freezing
    # leaves three legs in the air on a support triangle chosen at random by
    # whenever the operator released the key.
    if not self._settling:
        self._settling = True
        # remainder of this cycle, then one full cycle so both tripods land
        self._settle_remaining = (1.0 - self.phase) + 1.0

    step = dt / self.params.cycle_time
    self._settle_remaining -= step
    if self._settle_remaining <= 0.0:
        self.phase = 0.0          # all feet nominal, all down
        return
    self.phase = (self.phase + step) % 1.0
```

The stride is already shrinking to zero over the settle because §3's filter is decaying
`self.vx` toward zero, so the legs converge on their nominal positions rather than stopping
abruptly wherever they were. **§3 and §4 are one change; do them together or neither.**

`verify_gait.py` should gain a case: command a velocity, cut it at 20 different phases, and
assert that within two cycles all six feet are at nominal ± 1 mm and `stance_count() == 6`.

---

## 5. Worth taking later — body pose compensation inside the IK

`Ik::calculateIK` takes a full `hexapod_msgs::Pose` for the body and applies the Tait-Bryan
Z-Y-X rotation to every foot target before solving:

```cpp
Trig A = getSinCos(body.orientation.yaw + feet.foot[i].orientation.yaw);
Trig B = getSinCos(body.orientation.pitch);
Trig G = getSinCos(body.orientation.roll);
// ... full rotation matrix applied to (cpr_x, cpr_y, cpr_z)
```

The feet stay planted in the world while the body translates and rotates over them. In
their stack this drives IMU auto-levelling on uneven ground.

Two reasons to want it:

1. **Immediate, and relevant to our drift problem.** A small commanded body shift toward
   the supporting tripod each half-cycle keeps the COM comfortably inside the support
   polygon instead of near its edge. Statically stable hexapods that walk cleanly almost
   all do some version of this.
2. **Report material.** It is the natural place to later close an IMU loop, and it costs
   one optional argument.

Our version is a contained change, because `body_to_leg` already exists:

```python
def inverse_kinematics_body(geom, foot, body_pose=None):
    """
    body_pose: optional (x, y, z, roll, pitch, yaw) of the body relative to
    its nominal pose. Feet are specified in the WORLD-aligned frame and the
    body moves under them, so a non-zero pose shifts the robot over stationary
    feet rather than moving the feet.
    """
```

Apply the inverse body transform to `foot` before `body_to_leg`. `verify_ik.py`'s
round-trip test extends to it directly: FK → IK → FK with a fixed non-zero body pose must
still close to ~1e-16.

**Not urgent.** Do it after §2–§4 and only if the base is still not translating cleanly.

---

## 6. Worth taking — measure foot contact instead of inferring it

Every foot in their model carries a force sensor:

```xml
<plugin name="f3d_controller" filename="libgazebo_ros_f3d.so">
  <bodyName>foot_${side}${position}</bodyName>
  <topicName>${side}${position}_ground_feedback</topicName>
</plugin>
```

We currently infer stance from the gait clock (`foot_target` returns `in_stance`), which
tells us what we *asked for*, never what happened. When the body drifts we cannot presently
distinguish "a foot is slipping", "a foot never made contact", and "the solver is injecting
impulses" — the three causes named in `ARCHITECTURE.md §4` as producing identical symptoms.

Gazebo Classic's ROS 2 equivalent is a `<sensor type="contact">` on each foot link with
`libgazebo_ros_bumper.so` (or `libgazebo_ros_ft_sensor.so` at the foot joint). Six topics,
one per foot. Then a tool alongside `check_drift.sh`:

- commanded stance mask vs. measured contact mask, per tick → catches feet that miss
- vertical force sum vs. total weight → catches feet not carrying load
- tangential/normal force ratio vs. µ → **catches slip directly, which is the actual
  question**

This is the same principle as `ARCHITECTURE.md §7`: make the failure mode measurable rather
than guessable. It is the highest-information-per-hour item on this list even though it
changes no behaviour.

---

## 7. Explicitly do not copy

| Thing | Why not |
|---|---|
| Masses, inertias, mesh collisions | Non-physical; ours is correct. See §0. |
| `p: 100, i: 0, d: 0` gains | Tuned against 1e-5 kg links. Meaningless for our masses. |
| ±180° joint limits | Discards the mechanical constraints we derived. |
| Sinusoidal stance | Breaks non-slip; §2 gives a strictly better fix. |
| Integer tick counter as the phase clock | `cycle_period_++` per loop iteration makes gait speed depend on loop rate. Our `dt`-based phase is correct. |
| 4-DOF tarsus | Different mechanism. Our legs are 3-DOF. |
| Fatal `ros::shutdown()` on unreachable IK | Our `UnreachableTarget` + `check_reachable` at startup is better. |
| Monolithic gait+IK+servo node | The exact coupling our package split exists to avoid. |
| Spawning at `z=1` and dropping | We already pre-pose at stance height for good reason. |

---

## 8. Recommended order

1. **§4 stop sequencing + §3 command filter** — one change, addresses a real stability bug,
   low risk, both verifiable offline.
2. **§2 swing profile** — removes the contact-instant torque spike, ~6 lines, verifiable
   offline. Update `verify_gait.py`'s touchdown criterion to the world frame first.
3. **§6 contact sensing** — changes no behaviour, but converts the remaining drift question
   from argument to measurement before any gain tuning.
4. **§5 body pose in IK** — only if drift persists after the above.

Nothing here touches the URDF's physical parameters, the world file, or the PID gains.
Steps 1 and 2 are pure `hexapod_gait` changes and are provable with the existing offline
verifiers before Gazebo is involved at all.

### Minor, unrelated, noticed in passing

`gait_node.py` declares `stance_radius = 0.34` and `stance_height = -0.11` as parameter
defaults. Those are the **abandoned** stance values — `kinematics.py` documents them as
86% of full extension with the tibia horizontal, and `GaitParams` correctly defaults to
`0.28 / -0.18`. `walk.launch.py` overrides them to the good values, so this only bites when
running the node directly with `ros2 run`. Worth aligning the two defaults so the failure
cannot happen.

# Full explanation

Everything behind the slides: the kinematics derived step by step, the Gazebo
results with numbers, what every MATLAB plot means, and the future work.

Written to be studied, then said in your own words.

---

# PART 1 — THE KINEMATICS

## 1.1 What the two problems are

**Forward kinematics.** You know the three joint angles. Where is the foot?
Always has exactly one answer. Used for checking, and for plotting.

**Inverse kinematics.** You know where you want the foot. What should the
joint angles be? This is the one that runs on the robot, fifty times a second,
for all six legs. It can have zero, one or two answers.

## 1.2 The geometry trick that makes an exact solution possible

This is the single most important idea in the whole kinematics section, and
it is a **mechanism** argument, not a maths one. Say it slowly.

- The **coxa** joint rotates about a **vertical** axis.
- The **femur** and **tibia** joints rotate about **horizontal** axes that are
  **parallel to each other**.

Because those last two are parallel, the femur, the tibia and the foot are
locked into a **single vertical plane**. The coxa angle does not move the leg
within that plane. It only chooses **which** plane, by swinging the whole
assembly round.

So a three-dimensional positioning problem becomes:

1. one rotation to pick the plane, and
2. a flat two-link reach inside it.

A two-link planar reach is a triangle, and triangles are solved exactly with
the law of cosines. That is why we get a closed-form answer instead of having
to run a numerical solver.

**Why anyone should care:** a numerical solver takes an unpredictable number
of iterations and can fail to converge. Ours takes the same fixed number of
operations every single time and cannot fail. On a robot doing this 50 times a
second across six legs, that difference is the whole ball game.

## 1.3 Forward kinematics, derived

Work in the leg plane. Let `r` be how far the foot is from the coxa axis
measured outward, and `z` be how far it is below.

Walk out along the chain, adding each link's projection:

- After the coxa: you are `L₁` out, still level. Position `(L₁, 0)`.
- After the femur: the femur is at angle `θ₂` from horizontal, so it adds
  `L₂cos θ₂` outward and `−L₂ sin θ₂` vertically.
- After the tibia: the tibia's angle **relative to the ground** is
  `θ₂ + θ₃`, because joint angles are measured relative to the previous link.
  So it adds `L₃cos(θ₂+θ₃)` and `−L₃sin(θ₂+θ₃)`.

Adding up:

```
r = L₁ + L₂cos θ₂ + L₃cos(θ₂ + θ₃)
z =    − L₂sin θ₂ − L₃sin(θ₂ + θ₃)
```

Then lift back into three dimensions by rotating through the coxa angle:

```
x = r cos θ₁        y = r sin θ₁        z = z
```

**Why the minus signs on z.** Our axes are right-handed: x forward, y left,
z up. A rotation about the +y axis carries the +x direction toward −z. So a
**positive** femur angle moves the foot **downward**. We kept the clean
right-handed convention and documented the consequence, rather than flipping
an axis to make the sign feel nicer and creating a left-handed frame that
would break every standard rotation formula later.

If a question comes: *the angles are measured relative to the parent link, so
the tibia's angle in space is the sum θ₂ + θ₃. That is why the third term
carries the sum rather than θ₃ alone.*

## 1.4 Inverse kinematics, derived

Given a target foot position `(x, y, z)` in the leg frame.

### Step 1 — the coxa

The coxa is the **only** joint that can move the foot sideways. So its angle
is fixed entirely by the horizontal direction to the target:

```
θ₁ = atan2(y, x)
```

`atan2` rather than `arctan(y/x)` because it uses the signs of both arguments
to give the correct quadrant over the full ±180°, and does not divide by zero.

### Step 2 — reduce to two links

Move the origin out to the femur joint, which sits `L₁` along the plane:

```
r′ = √(x² + y²) − L₁          how far out the target is, from the femur joint
D  = √(r′² + z²)              straight-line distance, femur joint to foot
```

Now it is purely: two links of length `L₂` and `L₃` must reach a point `D`
away. Everything left is one triangle.

### Step 3 — the tibia, by the law of cosines

The triangle has sides `L₂`, `L₃` and `D`. The interior angle at the tibia
joint is `π − θ₃`, because `θ₃ = 0` means a **straight** leg. Applying the law
of cosines and simplifying:

```
cos θ₃ = (D² − L₂² − L₃²) / (2 L₂ L₃)
```

`arccos` returns a value in `[0, π]`, so `θ₃ = ±arccos(...)`. **Both are
correct** and put the foot in exactly the same place. See §1.6.

### Step 4 — the femur, as two angles added

The femur angle is built from two pieces:

```
β = atan2(−z, r′)     how far below horizontal the target lies
ψ = the angle between the femur and the line to the target
```

`ψ` comes from the same triangle, law of cosines again:

```
cos ψ = (L₂² + D² − L₃²) / (2 L₂ D)
```

and then

```
θ₂ = β − ψ
```

**The sign of ψ follows the elbow branch.** `ψ` is the angle between the femur
and the line to the foot, and the femur lies on the opposite side of that line
for the opposite elbow. Pairing `+ψ` with `+arccos` gives a leg that quietly
does not reach the target, and every forward-inverse round trip test fails.
That is exactly what the verification catches.

## 1.5 Reachability — when there is no answer at all

A solution exists only if the target lies inside the ring the two links can
sweep:

```
|L₂ − L₃| ≤ D ≤ L₂ + L₃
```

With our lengths, `L₂ = 116.6 mm` and `L₃ = 150.0 mm`:

```
33.4 mm ≤ D ≤ 266.6 mm
```

Too close and the leg cannot fold enough. Too far and it cannot stretch. Ask
for something outside that and `arccos` receives an argument outside `[−1, 1]`.

**We raise an explicit error rather than clamping.** Clamping silently returns
a wrong answer at the workspace edge and the leg tries to tear itself off. An
unreachable target is a bug in the **gait**, and the gait is where it must be
fixed, not papered over in the kinematics.

**Our operating point:** stance is `r = 280 mm`, `z = −180 mm`, which gives
`D = 222 mm`, or **83% of maximum reach**. The nearest joint limit is 60°
away. Comfortable margin on every axis.

## 1.6 The elbow choice — the best mechanical story in the deck

`arccos` gives two answers. Both place the foot identically. They differ in
how the leg folds, and the difference is **not cosmetic**.

| | Negative root | Positive root (chosen) |
|---|---|---|
| Femur | +92.9°, pointing almost straight down | +15.4°, reaching outward |
| Tibia | −67.8°, reaching outward, 25° below horizontal | +67.8°, dropping down, 83° below horizontal |
| Knee, horizontally inside the foot | **136 mm** | **18 mm** |
| Torque at the knee, 10.9 N per foot | **1.48 N·m** | **0.19 N·m** |

The load on the tibia joint is the foot force times the **horizontal** distance
from joint to foot. That distance is 136 mm one way and 18 mm the other, so
the same stance costs **nearly eight times** the knee torque depending purely
on which mathematical root you take.

Measured consequence of the wrong branch in simulation: the tibia sagged 24.3°
under the robot's own weight and the body sat 5 cm low.

There is a second argument, about conditioning. With the tibia only 25° below
horizontal, an error in tibia angle converts almost entirely into **body
height** error. At 83° it converts almost entirely into **horizontal foot
position**, which the gait corrects for anyway. So the chosen branch is not
just lower torque, it puts the error where it does least harm.

**And it produced a build requirement.** The femur must swing past vertical to
place a foot under the robot, because the 150 mm coxa puts the femur joint well
outboard. Its travel is −60° to +120°, not −90° to +90°, which means the femur
servo horn has to be mounted with an **offset at assembly**. That is a
simulation result driving a mechanical decision.

## 1.7 Velocity kinematics

Differentiate the forward equations with respect to time:

```
ṙ = −(L₂ sin θ₂ + L₃ sin θ₂₃) θ̇₂ − L₃ sin θ₂₃ θ̇₃
ż = −(L₂ cos θ₂ + L₃ cos θ₂₃) θ̇₂ − L₃ cos θ₂₃ θ̇₃          where θ₂₃ = θ₂ + θ₃
```

Written as a matrix this is the **Jacobian**, `ṗ = J θ̇`.

Two uses:

1. **Singularity check.** The leg is singular when fully straight (`θ₃ = 0`)
   or fully folded (`θ₃ = π`). At a singularity a small foot movement demands
   an enormous joint speed. Our stance sits at `θ₃ = 67.8°`, far from both.
2. **Feedforward.** We compute the expected joint velocity and hand it to the
   controller. A pure PID lags a moving target by however much position error
   is needed to generate the driving torque. Giving it the expected velocity
   lets it anticipate instead of chase.

## 1.8 Verification — how we know all of this is right

Take a grid of joint angles. Compute the foot position (forward). Feed that
position back through the inverse. Compute the foot position again.

The two must agree. **Worst disagreement: 3.5 × 10⁻¹⁶ m.** That is the
double-precision floating point limit, not a modelling error.

**We compare foot positions, not joint angles**, and that detail matters.
Because there are two valid elbow solutions, correct code can legitimately
return different angles from the ones you started with. Comparing angles would
report failures that are not failures.

---

# PART 2 — THE GAIT

## 2.1 Stride is derived, not chosen

A foot on the ground must not move relative to the ground. But we command foot
positions in the **body frame**, and the body is moving. So a foot that is
stationary on the ground must travel **backwards through the body frame** at
exactly the body's own speed.

A body-fixed point at `p` under linear velocity `v` and yaw rate `ω` moves at
`(vₓ − ω p_y, v_y + ω pₓ)`. The foot must move at exactly the negative of that
to stay planted, for the whole stance duration:

```
sᵢₓ = −(vₓ − ω_z p_iy) · T_st
sᵢᵧ = −(v_y + ω_z p_ix) · T_st          T_st = β · T_cycle,   β = 0.5
```

Three consequences worth saying aloud:

- **Stride is not a tuning knob.** Any other value violates non-slip.
- **Each leg gets a different stride when turning**, because a point further
  from the turn centre travels further. That falls straight out of the `ω`
  terms.
- **Walking, strafing and turning are one code path**, not three special
  cases. This single expression covers all of them.

At our demo speed, 0.07 m/s with a 1.0 s stance, stride is **70 mm**.

## 2.2 The swing profile, and the frame error we corrected

The foot in the air must return from the back of its stroke to the front,
clear the ground, and land without skidding.

- **Vertical:** raised cosine, `h(τ) = H(1 − cos 2πτ)/2`, `H = 45 mm`. Zero
  vertical speed at touchdown. Correct, and unchanged throughout.
- **Horizontal:** a cubic whose **end slopes match the stance velocity**.

We originally used a cycloid, chosen because its horizontal velocity is zero
at both ends. The reasoning was one reference frame out. **Zero relative to
the body means full body speed relative to the ground** — exactly the skid it
was meant to prevent. The requirement is that swing hands over to stance with
**no jump in velocity**, and stance moves at `−v`, so swing must arrive at
`−v`, not at 0.

The unique cubic through (0,0) and (1,1) with both end slopes equal to
`m = −(1−β)/β` is:

```
σ(τ) = (2m − 2)τ³ + (3 − 3m)τ² + m τ            m = −1 at β = 0.5
```

It dips slightly below 0 after lift-off and overshoots slightly above 1 before
touchdown — 4.4% of stride, entirely airborne. That excursion **is** the fix:
it is the foot tracking ground-fixed motion through both transitions.

## 2.3 Static stability

Two groups of three legs, exactly half a cycle apart. Duty factor 0.5, so at
every instant **exactly three feet** are down. Each group's triangle contains
the centre of mass. This is the fastest gait a hexapod can use while remaining
statically stable.

Slower options fall out of the same framework by changing only the phase
offset table: a **wave** gait moves one leg at a time (duty 5/6, five feet
down), a **ripple** gait sits between the two. That the offsets are a table
rather than hard-coded is why.

---

# PART 3 — GAZEBO SIMULATION RESULTS

These are measured, reproducible, and the ones to quote.

## 3.1 The setup being measured

Twenty-five links, eighteen actuated joints, up to six ground contacts making
and breaking every cycle. Physics at 1000 Hz, controller at 200 Hz, gait at
50 Hz. Ground truth body pose from a simulator plugin, so body motion is
measured rather than estimated.

## 3.2 Result 1 — the robot stands correctly under its own weight

On spawn, the gait node compares every measured joint angle against the
commanded stance pose:

```
lf_femur_joint   commanded +0.2694   measured +0.2345   error +0.0349 rad
lf_tibia_joint   commanded +1.1837   measured +1.2245   error −0.0408 rad
...
largest error across all 18 joints:  0.0413 rad  =  2.37°
```

**What it demonstrates:** the robot supports 3.39 kg on three legs, and the
worst joint deflects by 2.37° from commanded. That is real compliance from
gravity acting through the joints, and it is small. It also proves masses,
inertias and joint limits are all consistent — an inconsistent model does not
stand, it collapses or explodes.

## 3.3 Result 2 — the controller tracks what it is told

Sampled while walking:

```
                asked      achieved    shortfall
coxa          −0.0010      −0.0010      0.0000
femur         +0.2203      +0.2203     −0.0000
tibia         +1.2397      +1.2397      0.0000
```

**Agreement to four decimal places.** The control loop is doing its job
exactly. This is worth stating because it separates two things that look
identical from outside: a controller that is failing, and a controller that is
succeeding while the physics does something else.

## 3.4 Result 3 — the legs execute the commanded gait

Measured joint sweep during walking:

```
coxa    10.0°  peak to peak      the stride
femur   14.2°  peak to peak      the lift
```

And computed from the gait at demo settings (0.07 m/s, 2.0 s cycle):

```
coxa 11.1°     femur 25.6°     tibia 40.1°     forward walking
coxa 21.8°     femur 23.0°     tibia 27.0°     turning at 0.25 rad/s
```

Every one of these is far inside the joint limits (coxa ±60°, femur −60° to
+90°, tibia −60° to +130°).

## 3.5 Result 4 — three feet down, throughout

The stance pattern holds for the whole run. The support triangle never drops
below three feet, which is the stability condition, satisfied continuously
rather than on average.

## 3.6 Result 5 — the physics parameters had to be derived, not guessed

Worth presenting because it is genuine engineering content.

A contact behaves as a **spring** of stiffness `kp` against the mass it
supports. Its natural frequency is `ω = √(kp/m)`. An explicit integrator needs
roughly ten samples per period to stay stable.

At `kp = 10⁶` and about 0.57 kg per foot: `ω = √(10⁶/0.57) = 1325 rad/s`,
which is **211 Hz**. Sampled at 500 Hz that is 2.4 samples per oscillation, so
the contacts **ring**. Six ringing friction cones sum to a small, effectively
random force — and a robot standing perfectly still slowly rotates and slides
across the world. It looks like a friction bug. It is an aliasing bug.

Fixed at both ends: **1000 Hz** timestep, and `kp` lowered to **5 × 10⁴**,
which puts the contact at about 47 Hz, roughly 21 samples per period.
Penetration under load becomes 5.6 N / 5×10⁴ = **0.1 mm**, invisible.

`kd = 500` against `c_crit = 2√(kp·m) = 337` gives a damping ratio near 1.5,
slightly overdamped, so contacts settle without bouncing.

**Solver iterations = 100.** Gauss-Seidel propagates force one joint per
iteration along a chain. We have a four-link chain, six times over, sharing one
body. The default 20 iterations leaves large residual constraint error, which
appears as springy, sagging legs. Sagging is solver error, not a control
problem — do not chase it with controller gains.

## 3.7 Result 6 — what remains

Walking under the full dynamic simulation with the measured masses is not yet
complete. State it plainly and move on. The behaviour is characterised, and
the next-phase list says what happens about it.

---

# PART 4 — THE MATLAB PLOTS

Four figures, and each one answers a **design** question. That framing is what
makes them worth showing.

## 4.1 Foot path in XYZ, and the fore-aft view

**What is plotted:** the trajectory of a single foot through one complete step,
modelled as a Bézier curve. The dashed blue curve is the path, the red marker
is the foot.

**What to read off it:**

- The path is **closed** — it returns exactly to where it began. If it did not,
  the leg would drift a little further every step.
- It is **smooth everywhere**, with no corners. A corner would be an infinite
  acceleration, and therefore an impulsive load on the gearbox.
- The **stride length** and **lift height** are set by the control points of
  the curve. Those are the two numbers that define a step.

**Design question answered:** what shape should a step be, before we commit to
implementing anything?

## 4.2 Foot velocity against phase

**What is plotted:** the speed of the foot along its path, in mm/s, against
normalised phase from 0 to 1.

**Shape:** starts at about **174 mm/s**, falls to a minimum of about
**135 mm/s** at mid-phase, returns to 174. Symmetric, smooth, no
discontinuity anywhere.

**Design question answered:** **how fast must the servo be able to move?**
Peak foot speed divided by the effective link radius gives the required joint
angular velocity, and that has to sit inside the servo's rated no-load speed
with margin. This is one of the two numbers that select a servo.

## 4.3 Foot acceleration against phase

**What is plotted:** the second derivative, in mm/s², same phase axis.

**Shape:** about **345 mm/s²** at the ends, dropping to **300 mm/s²** at
mid-phase. The same symmetric U.

**Design question answered:** **what inertial load does the servo see on top
of holding the robot up?** Acceleration times the moving mass gives the
inertial force. The key qualitative point is that the curve is **continuous
and bounded** — no spikes. A spike here would mean a shock load, and shock
loads are what strip gears.

## 4.4 Foot force against phase

**What is plotted:** the force at the foot through the step, in newtons.

**Shape:** highest at the start and end of the phase, about **173 N**, lowest
in the middle at about **150 N**. It follows the acceleration curve, which is
what you expect: where the foot is accelerated hardest, the force is largest.

**Design question answered:** **what torque must the joint produce?** Force
times the moment arm gives joint torque, and that is the number that selects
the servo class. The static part of the load is the **10.9 N per foot** from
the mass budget (3.39 kg × 9.81 / 3 legs = 11.1 N); the rest is inertial.

> If asked exactly how the force is derived, say it is the loaded foot force
> through the trajectory from the MATLAB model, with the static component
> coming from the mass budget. Do not invent a derivation on the spot.

## 4.5 Joint angles against phase

**What is plotted:** three stacked traces, θ₁ coxa, θ₂ femur, θ₃ tibia, in
degrees, against phase. This is the inverse kinematics from Part 1 applied at
every point along the Bézier path.

**What to read off it:**

- **Femur and tibia are smooth**, sweeping roughly 30° and 12° respectively.
  Both sit comfortably inside the joint limits.
- **Every point on the path is reachable.** Nowhere does the curve break or go
  vertical, which is what would happen at a workspace boundary.
- These traces are literally **what the controller receives**.

**Design question answered:** is the trajectory we designed actually
achievable by this leg, and how much of the joint range does it consume?

**One point for you, not for the slide:** the coxa trace steps from about 180°
to 0° at phase 0.5. That is almost certainly an `atan2` **wrap-around**, not
real motion. Before this goes to a servo it must be unwrapped, or the joint
will try to rotate a full turn at that instant. Worth checking before hardware.

## 4.6 How MATLAB and the ROS implementation relate

Be precise if asked, because it is an easy place to overclaim.

MATLAB was the **design and analysis** environment: propose a foot trajectory,
differentiate it, and extract velocity, acceleration and force to size the
actuators. The ROS implementation is the **built system**: it uses a
polynomial swing profile whose end slopes are matched to the stance velocity,
for the non-slip reason in §2.2.

Both describe the same kind of motion — lift, swing forward, place, sweep back
— and both are analysed by the same inverse kinematics. The MATLAB study told
us what the leg has to be capable of. The implementation is what runs.

---

# PART 5 — FUTURE WORK

## 5.1 Immediate, next phase

**1. Complete the dynamic simulation with the measured masses.**
The gait, kinematics, model and controller are each verified independently.
The remaining work is the interaction between the floating base and the
contact solver. Route: complete the configuration matrix so the cause is
isolated by experiment rather than argument.

**2. Tune the effort controller gains.**
Now genuinely reachable, because the velocity feedforward term has been
activated. The intended approach is **gravity compensation**: the static
holding torque at every joint is computable in closed form from the current
pose and the known masses, so it can be fed forward, leaving the PID to
correct only the residual instead of generating the entire holding torque from
position error. This is what makes effort control practical rather than a
tuning fight, and it is a mechanics calculation, not a control heuristic.

**3. Foot contact sensing.**
Per-foot force sensors are implemented and currently disabled. Switching them
on converts three failure modes that look identical from outside — a foot
slipping, a foot never making contact, and solver error — into three separate
measurements:

- commanded stance mask vs measured contact → feet that miss
- sum of normal forces vs total weight → feet not carrying load
- tangential/normal force ratio vs µ → **slip, measured directly**

## 5.2 Hardware

**4. Servo driver.** The only component that has to be written to move from
simulation to the real robot. Because `hexapod_description` depends on nothing
and the controller interface is fixed, this replaces one plugin line. The
gait, the kinematics and the model do not change.

**5. Mass verification.** Present masses assume 100% infill. Weighing one
printed coxa and one printed tibia takes five minutes and makes every mass in
the model measured rather than derived.

**6. Assembly and calibration.** Including the femur servo horn offset that
came out of the elbow-branch analysis in §1.6.

## 5.3 Capability

**7. Body pose control.** Adding an optional body pose to the inverse
kinematics lets the body translate and rotate over stationary feet. Two uses:
shifting the centre of mass toward the supporting tripod during each step, and
later, levelling the body on uneven ground from an IMU.

**8. Terrain adaptation.** With contact sensing, a leg that touches down early
or late can adjust, which is the first real step beyond flat ground.

**9. Mapping and navigation.** The architecture already leaves room. This is
where the depth sensor and SLAM would attach, and it is the natural final
phase.

## 5.4 The honest summary of where this sits

Modelling, kinematics, gait and verification are **complete and proven**.
Simulation infrastructure is **complete**. Dynamic walking with the measured
model is **characterised but not finished**. Hardware is **architected but not
built**.

That is a good position at first review, and saying it in exactly those terms
is stronger than claiming more.

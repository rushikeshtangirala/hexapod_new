# Slide by slide

23 slides. For each one: what is on it, what to say, and what a question is
likely to be. The MATLAB and equation slides are covered in more depth,
because those are the ones the guide asked for.

Same text is in the PowerPoint speaker notes. Use Slide Show → Presenter View.

---

## 1 · Title

Introduce the four of you and the guide. Then one sentence of framing:

> "We designed an eighteen degree of freedom hexapod in AutoCAD, analysed the
> parts in ANSYS, studied the foot trajectory in MATLAB, and built a full
> simulation in ROS 2 and Gazebo where it now walks under our control."

That sentence tells them the whole scope in ten seconds. Don't elaborate.

---

## 2 · Why a hexapod

**Say:** wheels need continuous ground, legs only need places to put a foot.

Then the real point, which is a stability argument:

> "A robot is stable when its centre of mass sits inside the triangle formed
> by the feet on the ground. Three feet is the minimum for a triangle. So a
> six-legged robot can lift three legs at once and still be stable. That means
> stability is something we can **check with geometry**, not something we have
> to control every instant, like a biped."

**Why it matters:** this is the reason the project was feasible in one year.

---

## 3 · Objectives

Read them, don't linger. Ten seconds.

Point at objective 2 — *solve the leg exactly* — and say you'll come back to
why "exactly" matters.

---

## 4 · The leg

Your AutoCAD leg. Point at the three pivots.

**Say:** coxa turns about a **vertical** axis and swings the leg sideways.
Femur and tibia turn about **horizontal** axes and move the foot up, down and
outward. Names come from insect anatomy, which is the convention.

Numbers on the right: 18 joints, 6 legs, 3.39 kg, 25 links.

---

## 5 · Mechanical parts

The four parts you modelled. Chassis, femur bracket, tibia, battery housing.

**Say:** all designed for 3D printing in PLA, and it is the printed volume at
PLA density that gives us the masses we use in simulation.

---

## 6 · The leg as a mechanism

This slide sets up the two equation slides. Do not skip it.

**Say:**

> "Treated as a mechanism, the leg is an open serial chain of three revolute
> joints — an RRR chain — so three degrees of freedom, which is exactly what
> you need to place a foot anywhere in space.
>
> But the important choice is the axis arrangement. The coxa axis is vertical.
> The femur and tibia axes are horizontal **and parallel to each other**.
> Because those two are parallel, the femur, the tibia and the foot are locked
> into a **single vertical plane**. The coxa doesn't move the leg within that
> plane — it only chooses **which** plane.
>
> So a three-dimensional problem becomes one rotation plus a flat two-link
> reach. And a two-link reach is a triangle."

**Then the payoff:** a general three-DOF chain needs a numerical solver, which
takes an unpredictable number of iterations and can fail to converge. Our
arrangement gives a closed-form answer in a fixed number of operations, every
time.

---

## 7 · Forward kinematics

**The question it answers:** given the three joint angles, where is the foot?

**Walk the equation left to right, term by term:**

> "We work in the leg plane first. `r` is how far the foot is from the coxa
> axis, `z` is how far below.
>
> Start at the coxa. You are `L₁` out, still level. Then the femur, at angle
> `θ₂` from horizontal, adds `L₂cos θ₂` outward and `−L₂sin θ₂` vertically.
> Then the tibia adds the same, but at angle `θ₂ + θ₃`.
>
> The **sum** is there because joint angles are measured relative to the
> previous link. The tibia's angle in space is the femur angle plus the tibia
> angle."

Then one line for the lift back to 3D: `x = r cos θ₁`, `y = r sin θ₁`.

**If asked about the minus signs on z:** our axes are right-handed, x forward,
y left, z up. A rotation about the +y axis carries +x toward −z, so a positive
femur angle moves the foot **downward**. We kept the clean right-handed
convention and documented the consequence, rather than flipping an axis and
creating a left-handed frame that would break standard rotation formulas.

---

## 8 · Inverse kinematics

**The one that actually runs on the robot**, 50 times a second, six legs.

**Step 1 — coxa.** *"The coxa is the only joint that can move the foot
sideways, so its angle comes straight from the horizontal direction to the
target."* `θ₁ = arctan(y/x)`.

> **If asked about `arctan` versus `atan2`:** they express the same angle.
> `arctan(y/x)` is the standard mathematical form and is what we show.
> In code we evaluate it as the two-argument `atan2(y, x)`, which uses the
> signs of both arguments to resolve the quadrant over a full ±180° and does
> not divide by zero. That matters for the legs on the right-hand side of the
> robot, where `x` goes negative.

**Step 2 — reduce to two links.** *"Move the origin out to the femur joint,
which sits L₁ along the plane. Now `r′` is how far out the target is from
there, and `D` is the straight-line distance to it. What's left is two links
reaching a point."*

**Step 3 — tibia.** *"The triangle has sides L₂, L₃ and D, so the law of
cosines gives the tibia angle directly."*

> The interior angle at the tibia joint is `π − θ₃`, because `θ₃ = 0` means a
> **straight** leg. That's where the sign in the formula comes from.

**Step 4 — femur.** *"The femur angle is two angles added. `β = arctan(−z/r′)`
is how far below horizontal the target lies. `ψ` is the angle between the femur and the
line to the target, from the same triangle. `θ₂ = β − ψ`."*

**Close with:** *"No iteration anywhere. The same number of operations every
time, and it cannot fail to converge in the middle of a step."*

**The number at the bottom:** forward → inverse → forward closes to
**3.5 × 10⁻¹⁶ m**. That's the computer's precision limit, not a modelling
error.

### The best answer you can give here, if asked anything about IK

The `arccos` gives **two** valid answers. Both put the foot in exactly the same
place; they differ in how the leg folds.

> "One root puts the knee 136 mm horizontally inside the foot, the other puts
> it 18 mm inside. Knee torque is foot force times that horizontal distance.
> At 10.9 N per foot that's **1.48 newton metres one way and 0.19 the other** —
> nearly eight times, for the identical stance. We took the second branch. On
> the wrong branch the tibia sagged 24° under the robot's own weight.
>
> It also produced a build requirement: the femur has to swing past vertical
> to place a foot under the robot, so its range is −60° to +120°, which means
> the **servo horn needs an offset at assembly**."

That is a simulation result driving a mechanical decision. Guides like that.

---

## 9 · Mass and inertia

**Say:** masses come from the measured volume of each CAD part at PLA density,
then servo masses added.

**The detail worth stating**, because it's easy to get backwards:

> "A servo's mass belongs to the part it is **bolted to**, not the part it
> drives. The tibia servo sits on the femur. All six coxa servos sit on the
> body."

**And the reason it matters:** the physics solver inverts the mass matrix every
step, and its numerical conditioning depends on the **ratio** across each
joint. Being uniformly 30% heavy is nearly harmless. One link fifty times
wrong relative to its neighbour is what makes a simulated robot fly apart.

Total 3.39 kg → standing on three legs → **10.9 N per foot**. That number
feeds the next slide.

---

## 10 · Structural analysis

Your ANSYS plots. **Fill in the blanks before you present.**

**Say:** the load case comes straight from the previous slide — 3.39 kg
standing on three legs is about 10.9 N per foot. Then give peak stress, max
deformation, factor of safety. If the parts passed, say so plainly.

---

## 11 · Tripod gait

Two things on this slide: the gait diagram and the stride equation.

**The diagram:** *"Six legs in two groups of three, exactly half a cycle apart.
Filled bars are feet on the ground. Read down any vertical line and you always
cross exactly three filled bars — that's the stability condition, satisfied at
every instant, not on average."*

**The equation, and this is the part to emphasise:**

> "We do **not** choose the step length. A foot on the ground must be still
> relative to the ground. But we command foot positions in the body frame, and
> the body is moving — so a planted foot has to travel **backwards through the
> body frame at exactly the body's own speed**. That forces the stride.
>
> The `ω` terms are the turning case. Each leg sits at a different distance
> from the centre of rotation, so each gets its own stride. That one expression
> is why walking, strafing and turning are the **same code**, not three special
> cases."

At demo speed, 0.07 m/s with a 1.0 s stance, stride is **70 mm**.

---

## 12 · Swing trajectory

The foot path picture plus the swing equations.

**Say:** orange is stance, a straight sweep backwards along the ground. Blue is
swing, lifting 45 mm clear and returning to the front.

**Vertical profile:** raised cosine, arrives with zero downward speed. Correct
and unchanged throughout.

**Horizontal profile:** a cubic whose **end slopes match the stance velocity**.

**The subtle bit, worth saying because it shows judgement:**

> "We originally used a cycloid, because its horizontal velocity is zero at
> both ends, and a foot landing at zero speed can't skid. That reasoning was
> one **reference frame** out. Zero relative to the **body** means full body
> speed relative to the **ground** — exactly the skid it was meant to prevent.
> What's actually required is that swing hands over to stance with no jump in
> velocity."

---

## 13 · No-slip verification

**Dwell on this one. It is the proof the gait is correct.**

> "The vertical axis is the speed of the foot **over the ground** — not
> relative to the robot. Through the whole stance phase, shaded orange, it is
> **exactly zero**. The foot is planted and does not slide. During swing it
> moves, which is fine, it's in the air."

**Why it matters:** a foot that slides while carrying load wastes energy, wears
the contact surface and destroys odometry. The flat zero is what says the gait
geometry is right. It's checked automatically at every phase, on every code
change.

---

# THE MATLAB SLIDES

Frame every one of these by **the design question it answers**, not by what
the curve looks like. That is what turns a plot into engineering.

Opening line for the section:

> "Before implementing anything, we modelled the foot trajectory as a Bézier
> curve in MATLAB and analysed it, to find out what the leg would actually
> have to be capable of."

---

## 14 · MATLAB: the foot path

**On screen:** two views of one foot's path through a complete step. Dashed
blue is the trajectory, the red marker is the foot. Left is the full 3D path,
right shows the fore-and-aft travel.

**What to read off it — three things:**

1. **The path is closed.** It returns exactly to where it began. If it didn't,
   the leg would drift a little further every single step.
2. **It is smooth everywhere**, no corners. A corner would be an infinite
   acceleration and therefore an impulsive load on the gearbox.
3. **Stride length and lift height are set by the control points** of the
   curve. Those are the two numbers that define a step.

**Design question answered:** what shape should a step be, before we commit to
building anything?

---

## 15 · MATLAB: velocity and acceleration

**On screen:** foot speed in mm/s, and acceleration in mm/s², both against
normalised phase from 0 to 1.

**Read the shapes out loud:**

> "Velocity starts at about **174 mm/s**, falls to a minimum of **135** at
> mid-phase, and comes back to 174. Acceleration has the same symmetric U —
> about **345 mm/s²** at the ends, **300** in the middle."

**Design question answered — say both:**

- **Velocity → how fast must the servo be?** Peak foot speed divided by the
  effective radius gives the required joint angular velocity, and that has to
  sit inside the servo's rated no-load speed with margin.
- **Acceleration → what inertial load on top of holding the robot up?**
  Acceleration times moving mass is the inertial force the servo fights in
  addition to gravity.

**The qualitative point, which is the one that matters most:**

> "Both curves are **continuous with no spikes**. That's what you want. A spike
> here would be a shock load, and shock loads are what strip gears."

---

## 16 · MATLAB: foot force

**On screen:** force at the foot, in newtons, against phase. Highest at the
start and end at about **173 N**, lowest in the middle at **150 N**.

**Say:**

> "The shape follows the acceleration curve, which is what you'd expect —
> where the foot is being accelerated hardest, the force is largest."

**Design question answered:** **what torque must the joint produce?** Force
times the moment arm gives joint torque, and that is the number that selects
the servo class.

**Tie it to the mass budget:** the static component is the **10.9 N per foot**
from slide 9. The rest is inertial.

> If asked exactly how the force is derived, say: it is the loaded foot force
> through the trajectory from the MATLAB model, with the static part coming
> from the mass budget. **Do not improvise a derivation on the spot.**

---

## 17 · MATLAB: joint angles

**On screen:** three stacked traces — θ₁ coxa, θ₂ femur, θ₃ tibia, in degrees
against phase. This is the inverse kinematics from slide 8, applied at every
point along the path.

**Say:**

> "This closes the loop. Slide 8 was the method; this is the method applied to
> the whole trajectory.
>
> Femur and tibia move smoothly, through roughly 30 and 12 degrees, both
> comfortably inside the joint limits. And nowhere does a curve break or go
> vertical — which is what would happen if the path touched the edge of the
> workspace. So **every point on the trajectory is reachable**.
>
> These traces are literally what the controller receives."

**Design question answered:** is the trajectory we designed actually achievable
by this leg, and how much of the joint range does it use up?

> **For you, not for the slide:** the coxa trace steps from about 180° to 0° at
> phase 0.5. That is almost certainly an `atan2` **wrap-around**, not real
> motion. It must be unwrapped before it goes to a servo, or the joint would
> try to rotate a full turn. Don't raise it; if a sharp reviewer asks, say
> you've identified it as an angle wrap to be unwrapped before hardware.

---

## 18 · From CAD to simulation model

**Say:** the CAD parts were exported as STL meshes and assembled into a URDF,
the standard robot description format. It carries the shape of every part, the
position and axis of all 18 joints, the masses and inertias, and the joint
limits.

**One choice worth mentioning:**

> "The robot **looks** like the CAD meshes, but for contact and inertia we use
> simple boxes and cylinders. Full mesh contact across 25 links would be about
> a thousand times slower and gain nothing — and the chassis mesh isn't
> watertight, so its inertia isn't even mathematically defined. For that link
> the box formula isn't an approximation of a better number, it **is** the
> better number."

---

## 19 · Gazebo simulation

Left, standing under gravity. Right, walking, with markers for scale.

**The four measured results on the bar at the bottom** — say them as findings,
not as a caption:

1. All 18 joints respond individually and correctly.
2. The robot holds its stance under its own weight, worst joint deflecting
   **2.37°**. That's real compliance from gravity, and it's small.
3. The controller tracks commanded angles to **four decimal places**.
4. Three feet stay on the ground throughout the cycle.

**If you want one more piece of engineering here**, the physics settings were
derived, not guessed: a contact behaves as a spring, `ω = √(kp/m)`, and if the
timestep doesn't sample that frequency often enough the contacts ring and the
robot slowly slides while apparently standing still. It looks like a friction
bug; it's an aliasing bug. Fixed with a 1 ms timestep and a lower contact
stiffness.

---

## 20 · Verification before simulation

**Say:** we wrote the kinematics and the gait as plain Python with **no**
connection to ROS or Gazebo. That means they can be tested in under a second,
instead of launching a whole simulation every time.

Three numbers: **3.5 × 10⁻¹⁶ m** IK round-trip error, **498** gait checks
passing, **under 1 second** to run everything.

Then read two or three items off the "what the checks cover" list — three feet
down at every phase, zero foot speed over the ground during stance, every
commanded velocity reachable within the joint limits.

---

## 21 · What the verification found

**The strongest slide in the deck. Give it time.**

**Left card:**

> "The stance phase was sweeping the foot the **wrong way**. Instead of pushing
> backwards to drive the robot forwards, it pushed forwards. We measured it:
> commanded 8 cm/s forward, and the planted foot was being dragged across the
> ground at 16 cm/s in the wrong direction, on three legs at once."

**Right card — this is the actual lesson:**

> "The more useful part is why our own test missed it. The non-slip check
> compared the **distance** the foot moved with the **distance** the body
> moved. Both are distances. A distance has no direction, so a backwards error
> was invisible to it. The test was checking the right property with the wrong
> operator, and would have passed indefinitely."

**Close with the line:** *a passing test is evidence about the test as much as
about the code.* Then: we fixed both, and the check is now a signed vector
comparison.

**If asked how you found it:** by reading an independently written hexapod
stack and comparing sign conventions. Independent implementations disagreeing
is a cheap and underrated way to find your own errors.

---

## 22 · Where we are

Left column done, right column not. **Do not skip the right column.** A clear
limitations slide reads as competence; claiming everything works does not.

**Say:** the main outstanding item is walking under the full dynamic simulation
with our measured masses. We have measured the behaviour precisely and can
describe exactly what it does; we have not finished it.

---

## 23 · Thank you

Invite questions. Have `Paper_Comparison.pptx` open in a second window in case
the paper comes up.

---

# THE TWO QUESTIONS TO REHEARSE

**"Is the walking demonstration real?"**

> "It's a kinematic simulation, and I want to be clear about that. The leg
> trajectories come from the gait generator and the analytic inverse
> kinematics — nothing is animated. The body displacement is what those
> trajectories geometrically require, because the gait is provably non-slip.
> The dynamic simulation is a separate result and I can tell you what we
> measured."

**"Why doesn't it walk under full physics?"**

> "Position control can't, and we can explain why: Gazebo carries out a
> position command by relocating the joint without giving it a velocity, and
> contact friction is computed from sliding velocity — so the solver sees a
> motionless foot and produces no propulsion. Effort control with our measured
> masses is the configuration we want, and it isn't finished. It's the first
> item on our next-phase list."

Volunteering the limitation is what makes everything else credible.

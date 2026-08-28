# Lesson 4 — Differential-Drive Kinematics

> **Goal:** Understand the math that turns two wheel speeds into robot motion (and
> back). This is pure theory — but it's the exact theory the next lesson
> (`wheel_odometry`) and the `diff_drive_controller` implement.

## Where this lives in the repo

- The robot is a differential drive: `robonav_training_robot.urdf.xacro`
  (wheels 0.075 m radius, 0.34 m apart — Lesson 3).
- The inverse kinematics are handled by the `diff_drive_controller` configured in
  Lesson 3.
- In Lesson 5, you will create `wheel_odometry` and implement the forward
  kinematics from this lesson.

## What "differential drive" means

A **differential-drive** robot has two independently driven wheels on a common axle
plus a passive caster for balance. You steer it by driving the wheels at *different*
speeds (hence "differential"):

- Both wheels same speed forward → drive straight.
- Right wheel faster than left → curve left.
- Wheels equal and opposite → spin in place.

It also can't move sideways: it can only drive along the direction it faces and
turn. A robot whose motion is constrained like this — fewer ways it can instantly
move than the 3 ways it can be positioned (x, y, heading) — is called
**non-holonomic**. (A robot that *could* also strafe sideways, like one on
omni-wheels, would be *holonomic*.) Pure pursuit has to respect this in Lesson 11.

The whole behavior is captured by two physical constants from the URDF:

- **r = wheel radius = 0.075 m**
- **L = wheel separation = 0.34 m** (also called track width / wheel base)

## From wheel spin to ground distance

Wheel encoders measure **angular** position in radians. A wheel that turns by Δφ
radians rolls a ground distance of:

```
d = r · Δφ
```

So if the left wheel turned Δφ_L and the right turned Δφ_R since the last update:

```
d_left  = r · Δφ_L
d_right = r · Δφ_R
```

This is the first calculation your wheel odometry node will perform in Lesson 5.

## Forward kinematics: two wheel distances → robot motion

Given how far each wheel rolled in a small time step, how far did the *robot* move
and turn?

**Center distance** (how far the robot's midpoint advanced):

```
d_center = (d_left + d_right) / 2
```

It's the average because the body's center is halfway between the wheels.

**Heading change** (how much the robot rotated):

```
Δθ = (d_right − d_left) / L
```

Intuition: if the right wheel rolls farther than the left, the robot pivots
left (counter-clockwise, positive θ). Divide the difference by the wheel
separation `L` because a wider robot turns less for the same wheel difference.

Here's the actual derivation (it's short). When a diff-drive robot turns, both
wheels sweep the **same angle** Δθ about a shared turn center, but on circles of
different radius. If the center of the robot turns on a circle of radius `R`, the
two wheels (a distance `L/2` to each side) turn on radii `R + L/2` and `R − L/2`.
Arc length = radius × angle, so:

```
d_right = (R + L/2)·Δθ
d_left  = (R − L/2)·Δθ
```

Subtract: `d_right − d_left = L·Δθ`, which rearranges to **Δθ = (d_right − d_left)/L**.
(Add them instead and you recover `d_center = (d_left + d_right)/2 = R·Δθ`.) No
figure needed — it's just "arc = radius × angle" applied to two concentric circles.

> **Edge cases worth checking your understanding on:**
> - `d_left == d_right` → Δθ = 0 → straight line.
> - `d_left == −d_right` → d_center = 0, Δθ ≠ 0 → spin in place.

## Integrating pose: motion → new position

Now turn the per-step motion `(d_center, Δθ)` into an updated pose `(x, y, θ)`.

The naive ("first-order") update projects the whole move along the *old* heading:

```
x ← x + d_center · cos θ
y ← y + d_center · sin θ
θ ← θ + Δθ
```

This repo does slightly better with a **midpoint ("second-order") update**: it
projects along the heading at the *middle* of the step. Why the midpoint? The robot
turns gradually across the step, so its heading sweeps from `θ` to `θ + Δθ`; the
*average* direction it actually traveled is the heading halfway through, `θ + Δθ/2`.
Using that average instead of the starting heading is more accurate whenever the
robot is turning while moving:

```
θ_mid = θ + Δθ/2
x ← x + d_center · cos(θ_mid)
y ← y + d_center · sin(θ_mid)
θ ← normalizeAngle(θ + Δθ)
```

(That `normalizeAngle` is the helper from Lesson 2 — headings must stay wrapped.)
You will implement this update in the next lesson.

## Velocities (the other useful output)

Dividing the per-step motion by the elapsed time `dt` gives the robot's
**body-frame velocities**:

```
v (forward, m/s)   = d_center / dt
ω (turn rate, rad/s) = Δθ / dt
```

These two numbers, `v` and `ω`, are the only non-zero parts of a
`geometry_msgs/Twist` — the message `/cmd_vel` carries. (A `Twist` technically has
six fields — linear x/y/z and angular x/y/z — but for this planar robot only
`linear.x` = v and `angular.z` = ω are ever set; the rest stay zero.) The robot's
entire planar motion is `(v, ω)`.

## Inverse kinematics: `/cmd_vel` → wheel speeds

The reverse problem — "I want the robot to drive at forward speed `v` and turn rate
`ω`, what wheel speeds do I command?" — is what the **`diff_drive_controller`**
solves (you configured it in Lesson 3, you'll feed it in Lesson 11):

```
v_right = v + (ω · L / 2)      (linear speed of right wheel contact)
v_left  = v − (ω · L / 2)
```

then divide by `r` to get the wheel angular speeds. Pure pursuit (Lesson 11)
produces `(v, ω)`; the controller turns it into wheel commands; the simulator turns
those into motion; and `wheel_odometry` reads the motion back via the encoders.
That's the full loop.

## Why this isn't enough by itself (motivation for Lessons 5–9)

This math is *exact* only if wheels never slip, the radius and separation are
perfect, and the floor is flat. In reality:

- Wheels slip and skid (the differential model breaks).
- Real `r` and `L` differ slightly from the nominal values.
- Tiny per-step errors **accumulate** — integrate thousands of small errors and the
  estimate drifts away from truth.

So wheel odometry alone is a good *short-term* motion estimate that *drifts*
long-term. The fix is to fuse it with an IMU (EKF, Lesson 7) and correct it against
a map with a lidar (particle filter, Lesson 9). Keep that limitation in mind as you
implement the math next lesson.

## Hands-on (paper + pen)

1. The robot drives perfectly straight 1.0 m. What are `d_left`, `d_right`,
   `d_center`, `Δθ`? How many radians did each wheel turn (use r = 0.075)?
2. The right wheel rolls 0.10 m, the left rolls 0.08 m, over 0.1 s. Compute
   `d_center`, `Δθ` (use L = 0.34), then `v` and `ω`.
3. Starting at `(0, 0, 0)`, apply the midpoint update for the step in (2). Where is
   the robot now? Redo it with the naive update and compare.
4. You command `v = 0.3 m/s`, `ω = 0.5 rad/s`. Using inverse kinematics, first give
   the left/right wheel **contact speeds** in m/s (`v ± ωL/2`), then convert each to
   a wheel **angular speed** in rad/s (divide by `r`).

### Check your calculations

Try the problems before reading these results:

1. `d_left = d_right = d_center = 1.0 m`, `Δθ = 0`, and each wheel turns
   `1.0 / 0.075 ≈ 13.33 rad`.
2. `d_center = 0.09 m`, `Δθ = 0.02 / 0.34 ≈ 0.0588 rad`,
   `v = 0.9 m/s`, and `ω ≈ 0.588 rad/s`.
3. `θ_mid ≈ 0.0294 rad`, so the midpoint result is approximately
   `(x, y, θ) = (0.08996, 0.00265, 0.0588)`. The naive update gives
   `(0.09, 0, 0.0588)` and misses the small sideways displacement during the turn.
4. `v_left = 0.215 m/s` and `v_right = 0.385 m/s`, corresponding to wheel angular
   speeds of approximately `2.87 rad/s` left and `5.13 rad/s` right.

## Implementation handoff

Lesson 4 is paper math. In Lesson 5, you turn these equations into a new
`wheel_odometry` package. Use
[Lesson 5 implementation steps](implementation-steps/lesson-05-wheel-odometry.md)
when you are ready to code.

## Video supplement

- **Detailed and Correct Derivation of Kinematics Equations of Differential Drive Mobile Robot**
  Aleksandar Haber PhD

## Check yourself

- Convert a wheel's radian rotation to ground distance — what constant do you need?
- Derive `d_center` and `Δθ` from `d_left`, `d_right`, and `L`.
- Why is the midpoint integration more accurate than projecting along the old
  heading?
- What two numbers fully describe a diff-drive robot's planar velocity?
- Give three reasons this exact math still drifts on a real robot.

## Recap

A diff-drive robot is governed by two constants (**r**, **L**). **Forward
kinematics** turns wheel distances into `d_center` and `Δθ`, which integrate into a
pose `(x, y, θ)` and divide into velocities `(v, ω)`. **Inverse kinematics** turns
a desired `(v, ω)` back into wheel speeds. The model is exact only without slip, so
it drifts — motivating sensor fusion later. Next lesson you'll see every one of
these formulas in real C++.

➡️ **Next:** [Lesson 5 — Wheel Odometry](lesson-05-wheel-odometry.md).

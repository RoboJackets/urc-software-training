# Lesson 9 — Particle Filter (Monte Carlo Localization)

> **Goal:** Understand Monte Carlo Localization — representing belief with a cloud
> of particles, moving them with odometry, scoring them against the lidar via a
> likelihood field, and resampling — then implement the particle-filter motion
> update that moves every hypothesis using noisy odometry.

## Starter files for this lesson

- `src/particle_filter/src/particle_filter.cpp` — node, callbacks, top-level logic.
- `src/particle_filter/src/pf_motion_update.cpp` — motion update you complete.
- `src/particle_filter/src/pf_compute_score.cpp` — provided distance field and
  lidar measurement scoring.
- `src/particle_filter/src/pf_resample.cpp` — resampling + roughening.
- `src/particle_filter/src/pf_random_helpers.cpp` — RNG + angle wrap.
- `src/particle_filter/src/pf_tf_helpers.cpp` — TF lookups + `map→odom` broadcast.
- Headers in `include/particle_filter/`. Launch in `launch/`.

The algorithm is split across files on purpose — each file is one responsibility,
matching the repo's readability-first philosophy.

You do not rewrite the full particle filter from scratch. Follow
[Lesson 9 implementation steps](implementation-steps/lesson-09-particle-filter.md)
to fill in the motion update after reading this lesson. In the starter repo, the
files listed above already exist; only `applyMotionUpdate(...)` is incomplete.

## Why we need this

The EKF (Lesson 7) gives a smooth pose that still **drifts** — it has no external
reference. We have a **map** (Lesson 8) and a **lidar** (Lesson 6). If we can figure
out where on the map the robot must be for its lidar scan to match the walls, we can
correct the drift. That's **localization on a known map**, and the classic solution
is **Monte Carlo Localization (MCL)**, a.k.a. a **particle filter**.

> A note on the name: ROS's standard localizer is called **AMCL** ("Adaptive MCL,"
> which also varies the particle count over time). This repo implements plain MCL
> but publishes its result on the topic `/amcl_pose` so standard tools and RViz
> configs that expect that name just work — don't read "amcl" in the topic name as
> meaning this is the full AMCL package.

## The big idea: represent belief with particles

The EKF represented belief as a single Gaussian (one mean + covariance). A particle
filter represents belief as a **cloud of guesses** — hundreds of candidate poses,
each a **particle**:

```cpp
struct Particle { double x, y, theta; double weight; };
```

Each particle is a hypothesis "maybe the robot is *here*, facing *this* way," and
`weight` is how plausible it is. The robot's estimated pose is the weighted average
of the cloud. The advantage over a single Gaussian: a particle cloud can represent
*multiple* hypotheses at once ("I'm either by the left wall or the right wall") and
arbitrary, non-Gaussian shapes — essential when you're lost.

It's the same **predict / update** Bayes-filter loop as the EKF, just with a
different representation:

| Step | EKF (Lesson 7) | Particle filter |
| --- | --- | --- |
| Belief | one Gaussian | cloud of weighted particles |
| Predict | advance mean+cov by motion | move every particle by odometry + noise |
| Update | Kalman gain from covariance | reweight every particle by lidar likelihood |
| Extra | — | **resample**: drop bad particles, clone good ones |

## Step 0 — Initialization

When the map arrives, the filter finds all **free cells** (occupancy < 50) and
scatters particles uniformly across them, each with a random heading and equal
weight `1/N`. Starting only on free cells means no particle begins inside a wall.
(If you know the start pose, you'd instead cluster particles around it; this repo
defaults to spreading them.)

The constructor also creates a small local-prior particle set around `init_x`,
`init_y`, and `init_yaw` as a pre-map fallback. In the normal training launch, that
temporary set is replaced as soon as `/map` arrives, because map-based global
localization is the behavior this course demonstrates.

Key parameter: **`num_particles`** — more particles = more accurate but more CPU.
The standalone checkpoint in this lesson uses the code default of 250. Lesson 12's
integrated launch raises it to **2000** and supplies the final tuned noise and
likelihood values.

## Step 1 — Motion update

On each `/odometry/filtered` message (the EKF output — note the filter rides on top
of the fused odometry, not raw wheels), the filter:

1. Computes the odometry **delta** since last time: `dx, dy, dyaw` (yaw wrapped with
   `wrapAngle`, the Lesson 2 helper).
2. Converts that global delta into the robot's **local** frame, then applies it to
   **each particle** relative to *that particle's* own heading (the Lesson 2
   rotation pattern, per particle).
3. Adds **Gaussian noise** to each particle's move. Crucially the noise **scales
   with how far the robot moved** (`σ_x = x_noise · translation`, etc.): move more →
   spread more. This keeps the cloud honest — without added noise every particle
   would move identically and the cloud would never explore alternatives. (That
   failure mode has a name: **particle deprivation** — the cloud collapses to too few
   distinct hypotheses and can't recover.)
4. Optionally injects a small percent of **random particles** (`random_particle_percent`)
   scattered on free cells — a recovery mechanism for the **"kidnapped robot"
   problem** (the classic scenario where the robot is physically picked up and set
   down somewhere else, so the whole estimate is suddenly wrong; random particles can
   rediscover the true location).
5. Publishes the cloud on **`/particle_cloud`** — a `PoseArray`, which is just a list
   of (position, orientation) pairs, one per particle — for visualization in RViz.

Relevant params: `x_noise`, `y_noise`, `theta_noise`, `random_particle_percent`.

This is the learner-owned portion of the particle filter. The callback already
computes the odometry delta in the body frame; you implement the short loop that
adds scaled noise and rotates that delta by each particle's heading.

## Step 2 — The measurement model: a likelihood field

Now the heart of it: given a particle's pose, **how well does the real lidar scan
match the map?** Doing real ray-casting for every beam of every particle is too
slow. Instead the repo precomputes a **likelihood field** (a.k.a. distance field):

> For every cell, store the distance to the **nearest occupied cell**.

This is built once when the map arrives using a **Euclidean Distance Transform** — a
standard image algorithm that computes every cell's distance to the nearest occupied
cell in two linear passes (one over columns, one over rows), so it's fast even on a
big grid. You can treat it as a black box inside `pf_compute_score.cpp`: precompute
once, and afterward "how far is this point from the nearest wall?" is an O(1) table
lookup. (The video supplement walks through how the two-pass transform works if
you're curious.)

### Scoring a particle (`score`)

This measurement-scoring path is provided in the starter. Read and trace it, but
you are not expected to implement its likelihood-field and log-probability
numerics in this introductory course.

For a given particle and each lidar beam (skipping beams with `beam_stride` for
speed — default 3, i.e. every 3rd beam):

1. **Project the beam's endpoint** into the map: account for the lidar's mounting
   offset (`base → lidar`, looked up via the provided TF helper), the particle's
   pose, the beam angle, and the measured range. (This is the Lesson 6 "beam *i* hit
   point" math, transformed by the particle's pose.)
2. **Look up** the distance `d` from that endpoint to the nearest wall in the
   likelihood field.
3. **Convert distance to a probability** with a mixture model:
   - a **Gaussian hit** term — the closer the endpoint lands to a real wall, the
     higher the probability:
     `z_hit · N(d; 0, σ_hit²)`,
   - a **uniform random** term `z_rand · 1/range_max` — accounts for unexpected
     obstacles and sensor noise so one bad beam can't zero out a good particle.
4. **Multiply** the per-beam probabilities to get the particle's total likelihood.
   In practice this is done by **adding their logarithms instead of multiplying the
   raw values** (converting back only at the end — the "log-sum-exp trick"), because
   multiplying hundreds of tiny numbers would underflow to zero in floating point.

A particle whose pose makes the scan's endpoints land right on the map's walls gets
a high score; a misaligned particle gets a low score.

Params: `z_hit` (default 0.7), `z_rand` (0.3), `sigma_hit` (0.10 m), `beam_stride`.
The full-stack launch overrides these to the tuned demo values listed above.

After scoring, weights are **normalized** to sum to 1 (again via log-sum-exp). If
*all* particles score terribly (total weight ≈ 0), the filter resets to uniform
weights rather than dividing by zero.

## Step 3 — Resampling

Over time a few particles accumulate most of the weight and the rest are dead
weight. **Resampling** rebuilds the cloud: draw `N` new particles from the old set
*with probability proportional to weight*, so high-weight particles get copied many
times and low-weight ones vanish.

Details in this repo:

- **When:** only resample if the **effective sample size** drops below half the
  particles: `N_eff = 1 / Σ(weightᵢ²)`, resample if `N_eff < 0.5·N`. Resampling too
  often needlessly destroys diversity.
- **How:** **systematic resampling** — one random offset, then `N` evenly spaced
  picks through the cumulative weight distribution. It's lower-variance and faster
  than naive multinomial sampling.
- **Roughening:** after resampling, add tiny Gaussian jitter (~half a cell in x/y,
  ~0.01 rad in θ) so the many clones of a winning particle don't collapse into the
  exact same point. This preserves the cloud's ability to keep exploring.

## Step 4 — Estimate and broadcast

After the measurement update, the filter computes the estimated pose:

- weighted mean of x and y,
- **circular** mean of heading (`atan2(Σ w·sinθ, Σ w·cosθ)` — you can't just average
  angles, Lesson 2),
- a weighted covariance for uncertainty.

It publishes this on **`/amcl_pose`** as a `PoseWithCovarianceStamped` — read the
type name as its parts: a **pose** + a **covariance** (the uncertainty, Lesson 7) +
**stamped** (carrying a timestamp and frame).

Then the key output — the **`map → odom`** transform (`broadCastMapToOdomTf`). The
filter knows `map → base` (the estimated pose) and `odom → base` (from odometry), so
it solves:

```
map→odom = (map→base) · inverse(odom→base)
```

exactly the composition/inversion from Lesson 2. Broadcasting this completes the TF
tree (`map → odom → base_footprint → …`) and is what actually corrects the EKF's
drift: the smooth `odom→base` keeps flowing, and `map→odom` nudges occasionally to
keep the robot globally aligned to the map.

## Topics summary

| Direction | Topic | Type | Role |
| --- | --- | --- | --- |
| sub | `/odometry/filtered` | `nav_msgs/Odometry` | motion model |
| sub | `/lidar/scan` | `sensor_msgs/LaserScan` | measurement model |
| sub | `/map` | `nav_msgs/OccupancyGrid` | build likelihood field, seed particles |
| pub | `/amcl_pose` | `PoseWithCovarianceStamped` | estimated pose |
| pub | `/particle_cloud` | `PoseArray` | visualization |
| TF | `map → odom` | transform | the global drift correction |

## Hands-on

First complete
[Lesson 9 implementation steps](implementation-steps/lesson-09-particle-filter.md).

Start the simulator, wheel odometry, and EKF exactly as in Lesson 7. Also start the
map server from Lesson 8. Then add the particle filter in a new sourced terminal:

```sh
ros2 launch particle_filter particle_filter.launch.py
```

This is the localization stack you have built so far. In another terminal, drive
with teleop and inspect:

```sh
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom    # watch the correction
```

In RViz:

1. Add a **PoseArray** display on `/particle_cloud`. On startup you'll see particles
   scattered widely; as you drive, watch them **converge** into a tight cluster on
   the robot as the lidar disambiguates the pose. This is the single most satisfying
   thing to watch in the whole course.
2. Set Fixed Frame to `map`. Note the robot occasionally **hops** as `map→odom`
   updates — that's the correction (vs. the smooth `odom` frame from the EKF).
3. Increase the existing `random_particle_percent` value in
   `particle_filter.launch.py`, relaunch, and watch more random hypotheses remain
   scattered while the main cloud converges.

## Video supplement

- [Particle Filters | Robot Localization (Bot Field)](https://www.youtube.com/watch?v=ydC0mE0ZYSA)

## Check yourself

- How does a particle cloud represent belief differently from the EKF's Gaussian,
  and when does that matter?
- Why precompute a likelihood field instead of ray-casting per particle?
- Why does the motion noise scale with distance traveled?
- What is `N_eff` and why resample only when it's low? Why roughen afterward?
- Derive how `map→odom` is computed from `map→base` and `odom→base`.

## Recap

**MCL** represents the robot's belief as a cloud of weighted **particles**. Each
cycle: **move** particles by odometry + scaled noise (predict), **reweight** them by
how well their projected lidar endpoints land near walls in a precomputed
**likelihood field** (update), and **resample** when diversity drops, with
roughening to avoid collapse. The weighted mean is published on `/amcl_pose`, and
the filter broadcasts the **`map → odom`** transform that corrects the EKF's drift —
completing the localization stack. Now the robot truly knows where it is; time to go
somewhere.

➡️ **Next:** [Lesson 10 — A* Path Planning](lesson-10-a-star-planning.md).

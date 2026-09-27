# Project 9 — Particle Filter Motion Update

> **Goal:** Move every particle using the robot's odometry change and add noise
> that grows with motion.

Edit only:

```text
src/particle_filter/src/pf_motion_update.cpp
```

Add `#include <cmath>` below the package header include.

Use the [motion-update scaffold](../reference/project-09-particle-filter.md#motion-update-scaffold).
The caller already provides:

- `dx_body`, `dy_body`: translation in the robot frame
- `dyaw`: heading change
- `translation`, `rotation`: motion magnitudes used to scale noise

The class already provides `particles_`, `gaussianNoise(...)`, `wrapAngle(...)`,
and the three noise parameters.

## Implement The Function

1. For the first TODO, calculate the noise standard deviations once. 
- We want to calculate sigma_x, sigma_y and sigma_theta at this step. 
- Each sigma is a standard deviation, it controls the spread of random values.
- The standard deviation depends on how uncertain (or noisy) a movement is, and the maginutude of the movement.
- Think about how you can calculate these values given the parameters for the applyMotionUpdate member function.

2. For the second TODO, loop through `particles_`. For each particle, create separate noisy values:
- We want to calculate noisy_dx, noisy_dy, and noisy_dyaw at this step.
- We're given the robot's odometry changes (dx_body, dy_body, dyaw)
- We’ve also calculated the three sigma values, which control the spread of the random errors we’ll generate.
- We use gaussianNoise(...) along with the corresponding sigma value to generate the random error for each movement component of each particle.
- These noisy delta values depend on the current odom values of the robot, and gaussianNoise(...).

3. Rotate the noisy translation by that particle's `theta` using the reference
   lines, then add it to `particle.x` and `particle.y`.

4. Update `particle.theta` and wrap it with `wrapAngle(...)`.

Do not change particle weights in this function.

## Build And Check

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select particle_filter
source install/setup.bash
```

Then follow the Lesson 9 hands-on checkpoint. Driving should move the particle
cloud along each particle's heading and spread it slightly. If particles always
move along map x, recheck the rotation.

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

1. Calculate the noise standard deviations once:

   ```text
   sigma_x = x_noise_ * translation
   sigma_y = y_noise_ * translation
   sigma_theta = theta_noise_ * rotation
   ```

2. Loop through `particles_`. For each particle, create separate noisy values:

   ```text
   noisy_dx = dx_body + gaussianNoise(sigma_x)
   noisy_dy = dy_body + gaussianNoise(sigma_y)
   noisy_dyaw = dyaw + gaussianNoise(sigma_theta)
   ```

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

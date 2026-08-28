# Lesson 9 Implementation - Particle Filter Motion Update

The full particle filter is intentionally not rewritten from scratch. The node,
subscriptions, initialization, lidar scoring, weight normalization, resampling,
random-particle recovery, publishing, and TF are provided.

Students implement the prediction step: move every particle using the change in
odometry, with motion-scaled Gaussian noise.

## Target File

Fill in:

```text
src/particle_filter/src/pf_motion_update.cpp
```

The declaration is already provided in:

```text
src/particle_filter/include/particle_filter/particle_filter.hpp
```

The package already builds while this function is a no-op.

## Inputs Already Computed For You

`odomCallback(...)` already handles ROS messages, timestamps, TF fallback, the
first-message baseline, and the world-to-body rotation. It calls:

```cpp
applyMotionUpdate(dx_body, dy_body, dyaw, translation, rotation);
```

The arguments mean:

- `dx_body`: forward odometry change in meters
- `dy_body`: leftward odometry change in meters
- `dyaw`: heading change in radians
- `translation`: total distance moved in meters
- `rotation`: absolute heading change in radians

The following helpers and members are also provided:

- `particles_`: every current particle
- `x_noise_`, `y_noise_`, `theta_noise_`: noise scale parameters
- `gaussianNoise(stddev)`: zero-mean Gaussian sample
- `wrapAngle(angle)`: wraps an angle to `[-pi, pi]`

## Part 1: Scale Noise With Motion

Calculate three standard deviations:

```text
sigma_x = x_noise * translation
sigma_y = y_noise * translation
sigma_theta = theta_noise * rotation
```

No movement produces no added noise. More movement spreads the particle cloud
farther because odometry uncertainty grows with distance and rotation.

## Part 2: Add Noise To The Body-Frame Delta

For each particle, make a noisy copy of the odometry change:

```text
noisy_dx = dx_body + gaussianNoise(sigma_x)
noisy_dy = dy_body + gaussianNoise(sigma_y)
noisy_dyaw = dyaw + gaussianNoise(sigma_theta)
```

Do not change the particle's weight in the motion update. Lidar scoring owns the
weights.

## Part 3: Move In Each Particle's Direction

The delta is expressed in the robot's body frame, but each particle has its own
hypothetical heading. Rotate the same noisy body-frame movement by
`particle.theta` before adding it:

```text
world_dx = cos(theta) * noisy_dx - sin(theta) * noisy_dy
world_dy = sin(theta) * noisy_dx + cos(theta) * noisy_dy
```

Then update:

```text
particle.x += world_dx
particle.y += world_dy
particle.theta = wrapAngle(particle.theta + noisy_dyaw)
```

This is the complete learner-owned algorithm. Random-particle injection remains
provided in `odomCallback(...)`, after this function returns.

## Build

```sh
cd /workspace
colcon build --symlink-install --packages-select particle_filter
source install/setup.bash
```

## Acceptance Checks

Run the full stack after Lesson 12 wiring and display `/particle_cloud` in RViz.

Expected behavior:

- before this implementation, odometry does not move the particle cloud
- driving forward moves every particle along its own heading
- turning changes particle headings
- the cloud spreads slightly while moving because of Gaussian noise
- lidar scoring subsequently concentrates the cloud around poses that match the map

If particles move along fixed map x regardless of their headings, the body-to-map
rotation is missing. If the cloud spreads while stationary, confirm that the
provided deadband is still present in `odomCallback(...)` and that noise is scaled
by `translation` and `rotation`.

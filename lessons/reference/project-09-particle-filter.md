# Project 9 Reference — Particle Filter Motion Update

Use this incomplete function with the Project 9 instructions.

## Motion-Update Scaffold

```cpp
void ParticleFilter::applyMotionUpdate(
  double dx_body, double dy_body, double dyaw,
  double translation, double rotation)
{
  // TODO: calculate sigma_x, sigma_y, and sigma_theta

  for (auto & particle : particles_) {
    // TODO: sample noisy_dx, noisy_dy, and noisy_dyaw

    const double c = std::cos(particle.theta);
    const double s = std::sin(particle.theta);
    const double world_dx = c * noisy_dx - s * noisy_dy;
    const double world_dy = s * noisy_dx + c * noisy_dy;

    particle.x += world_dx;
    particle.y += world_dy;
    particle.theta = wrapAngle(particle.theta + noisy_dyaw);
  }
}
```

Call `gaussianNoise(...)` separately for each particle. Do not change weights.

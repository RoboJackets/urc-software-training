# Project 11 Reference — Pure Pursuit

Use these incomplete pieces with the Project 11 instructions.

## Wrapper Parameters

```cpp
const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
const auto cmd_vel_topic =
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
```

## Lookahead Scaffold

```cpp
std::size_t lookahead_index = path.size() - 1;

for (std::size_t i = nearest_index; i < path.size(); ++i) {
  const double distance =
    std::hypot(path[i][0] - robot_x, path[i][1] - robot_y);
  if (distance >= lookahead_distance) {
    lookahead_index = i;
    break;
  }
}
```

## World-To-Robot Transform

```cpp
const double dx = path[lookahead_index][0] - robot_x;
const double dy = path[lookahead_index][1] - robot_y;

const double x_r =
  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
const double y_r =
  -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
```

## Command Shape

```cpp
PursuitCommand command{0.0, 0.0, false};

if (x_r < 0.0) {
  // TODO: slow forward command and turn toward the point
} else {
  const double distance = std::hypot(x_r, y_r);
  const double curvature =
    (distance > 1e-6) ? (2.0 * y_r) / (distance * distance) : 0.0;

  // TODO: apply goal slowdown
  // TODO: set linear and clamped angular command
}

return command;
```

Use `normalizeAngle(std::atan2(y_r, x_r))` when choosing the turn direction for
a point behind the robot.

# C++ Algorithm Patterns Used In This Course

Use this page when the robotics idea makes sense but the C++ container or math
syntax does not.

## Loop Over A Vector

Read-only values:

```cpp
for (const auto & value : values) {
  // read value
}
```

Values you need to modify:

```cpp
for (auto & value : values) {
  value.weight = 1.0;
}
```

When the index matters:

```cpp
for (std::size_t index = 0; index < values.size(); ++index) {
  const auto & value = values[index];
}
```

## Check Before Indexing

```cpp
if (index >= values.size()) {
  return;
}

const auto & value = values[index];
```

When an index begins as a signed `int`, reject negative values before converting
to `std::size_t`.

## Common Math Helpers

```cpp
#include <algorithm>
#include <cmath>
#include <limits>

const double distance = std::hypot(dx, dy);
const double bounded = std::clamp(value, minimum, maximum);
const double positive_or_negative = std::copysign(magnitude, sign_source);
const bool usable = std::isfinite(value);
const double infinity = std::numeric_limits<double>::infinity();
```

Angles use radians. `std::sin`, `std::cos`, and `std::atan2` all work in radians.

## Rotate A 2D Vector

Body-frame `(forward, left)` movement into a world frame at heading `yaw`:

```cpp
const double world_x =
  std::cos(yaw) * forward - std::sin(yaw) * left;
const double world_y =
  std::sin(yaw) * forward + std::cos(yaw) * left;
```

World-frame movement into the body frame uses the inverse rotation:

```cpp
const double forward =
  std::cos(yaw) * world_x + std::sin(yaw) * world_y;
const double left =
  -std::sin(yaw) * world_x + std::cos(yaw) * world_y;
```

## Lowest-Value Priority Queue

`std::priority_queue` normally returns the largest item. A* needs the entry with
the lowest score:

```cpp
#include <queue>
#include <vector>

struct Entry
{
  double score;
  int index;
};

const auto lowest_score_first =
  [](const Entry & left, const Entry & right) {
    return left.score > right.score;
  };

std::priority_queue<
  Entry,
  std::vector<Entry>,
  decltype(lowest_score_first)> queue(lowest_score_first);

queue.push({3.0, 12});
const Entry current = queue.top();
queue.pop();
```

The `>` comparison is intentional: it reverses the normal max-heap ordering.

## Initialize Per-Cell Storage

```cpp
const std::size_t cell_count = grid.cellCount();

std::vector<double> cost(
  cell_count, std::numeric_limits<double>::infinity());
std::vector<int> predecessor(cell_count, -1);
std::vector<bool> visited(cell_count, false);
```

All three vectors use the same flat cell index.

## Reconstruct A Predecessor Chain

```cpp
std::vector<int> reversed;
int index = goal_index;

while (index != -1) {
  reversed.push_back(index);
  index = predecessor[static_cast<std::size_t>(index)];
}

return std::vector<int>(reversed.rbegin(), reversed.rend());
```

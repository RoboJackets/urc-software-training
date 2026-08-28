# Project 10 Reference — A* Planning

Use these incomplete pieces with the Project 10 instructions.

## Wrapper Parameters

```cpp
const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
```

## Queue Scaffold

```cpp
auto heuristic = [&](int col, int row) {
  const double dc = static_cast<double>(col - goal_col);
  const double dr = static_cast<double>(row - goal_row);
  return std::hypot(dc, dr) * grid.resolution;
};

struct Entry
{
  double f;
  int col;
  int row;
};

auto cmp = [](const Entry & a, const Entry & b) {return a.f > b.f;};
std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> open(cmp);

open.push({heuristic(start_col, start_row), start_col, start_row});
```

## Search Loop Scaffold

```cpp
while (!open.empty()) {
  const Entry current = open.top();
  open.pop();

  const std::size_t current_index = grid.index(current.col, current.row);
  if (closed[current_index]) {
    continue;
  }
  closed[current_index] = true;

  if (current_index == goal_index) {
    // TODO: reconstruct and return the path
  }

  for (int neighbor = 0; neighbor < 8; ++neighbor) {
    // TODO: coordinates, validity checks, move cost, and relaxation
  }
}
```

## Relaxation Shape

```cpp
if (tentative_g < g_score[neighbor_index]) {
  g_score[neighbor_index] = tentative_g;
  came_from[neighbor_index] = static_cast<int>(current_index);
  open.push({tentative_g + heuristic(neighbor_col, neighbor_row),
             neighbor_col, neighbor_row});
}
```

## Reconstruction Shape

```cpp
std::vector<int> reversed;
int index = static_cast<int>(goal_index);

while (index != -1) {
  reversed.push_back(index);
  index = came_from[static_cast<std::size_t>(index)];
}

return std::vector<int>(reversed.rbegin(), reversed.rend());
```

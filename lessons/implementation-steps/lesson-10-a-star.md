# Lesson 10 Implementation - A* Search

Students implement the pure A* algorithm. The ROS wrapper, map subscription,
blocked-mask construction, obstacle inflation, TF lookup, and path publishing can
be provided as starter code.

The split is intentional: `a_star_search.cpp` is ordinary C++ with no topics or
callbacks. The wrapper converts ROS data into a `GridMap` and flat blocked mask,
calls the search, then converts returned cells into `nav_msgs/msg/Path`.

## Keep These References Open

- [priority queues, safe indexing, and path reconstruction](../reference/cpp-algorithm-patterns.md)
- [ROS-free `.hpp`/`.cpp` core pattern](../reference/cpp-node-patterns.md#ros-free-core-pattern)
- [`OccupancyGrid` and `Path` fields](../reference/messages-and-qos.md)
- [parameter and TF-listener patterns](../reference/cpp-node-patterns.md)
- [package-specific build and source commands](../reference/terminal-commands.md#build-and-source)

Take only the data-structure or ROS pattern needed for each TODO and adapt it to
the planner's existing types and requirements.

## Target Files

Fill in:

```text
src/a_star_planner/src/a_star_search.cpp
```

Use the declarations in:

```text
src/a_star_planner/include/a_star_planner/a_star_search.hpp
```

Starter code should provide:

- the `astarSearch(...)` function signature
- the `GridMap` type
- a `blocked` vector indexed by `grid.index(col, row)`
- small helper functions such as `isBlocked(...)`, if desired

In this starter, input validation, `isBlocked`, the score/predecessor/closed
arrays, and start g-score are already written. You implement the heuristic,
priority-queue entry/comparator/setup, pop/close loop, goal reconstruction, and
neighbor relaxation. The function returns an empty path until those blocks are
completed, so the package still builds safely.

Before the algorithm, complete one small ROS wrapper checkpoint in
`a_star_planner.cpp`: declare string parameters `map_topic`, `goal_topic`, and
`plan_topic` with the course defaults, then use those values when creating the two
subscriptions and publisher. Keep the map's reliable, transient-local QoS. Build
once here so ROS plumbing errors stay separate from A* errors.

Read `GridMap::inBounds`, `index`, `worldToMap`, and `mapToWorld` in
`robonav_training_common` before implementing the search. Inside the core,
represent a cell consistently as `(col, row)` and convert it to a flat index only
for vector access. The repository uses row-major indexing:

```text
index = row * width + col
col = index % width
row = index / width
```

## Implement The Algorithm

Students write:

- Euclidean heuristic
- an `Entry` struct or equivalent priority-queue item
- priority queue ordered by lowest `f = g + h`
- initial start entry
- 8-connected neighbor iteration
- diagonal move cost of `resolution * sqrt(2)`
- orthogonal move cost of `resolution`
- corner-cut prevention for diagonal moves
- stale-entry handling
- path reconstruction from goal back to start
- empty path when no route exists

Implement it in this order:

1. Read the provided validation and the `g_score`, `came_from`, and `closed`
   initialization.
2. Write a Euclidean heuristic that returns distance in meters.
3. Define the queue entry and lowest-f-first priority queue, then push the start with
   `f = heuristic(start)`.
4. Repeatedly pop the smallest-f entry. If its cell is already closed, it is a
   stale duplicate, so skip it; otherwise close it.
5. If it is the goal, follow `came_from` back to `-1`, reverse that list, and
   return it. Include both start and goal.
6. Visit all eight neighbors. Reject blocked/out-of-bounds neighbors and closed
   neighbors. For a diagonal, also require both adjacent orthogonal cells to be
   free so the robot cannot squeeze through an obstacle corner.
7. Compute `tentative_g = current_g + move_cost`. Only when this improves the
   neighbor's g-score should you update its predecessor and push a new queue
   entry.
8. Return `{}` if the queue empties before reaching the goal.

`std::priority_queue` is a max-heap by default. Its comparator therefore needs to
place the entry with the **lowest** f-score on top (for example, compare with
`a.f > b.f`). It is normal for the same cell to appear more than once because C++
priority queues do not provide a simple decrease-key operation; closed/stale
handling makes those duplicates safe.

This is enough scaffolding to establish the open set without giving away the
neighbor relaxation:

```cpp
struct Entry
{
  double f;
  int col;
  int row;
};

auto lower_f_first = [](const Entry & a, const Entry & b) {
  return a.f > b.f;
};
std::priority_queue<
  Entry, std::vector<Entry>, decltype(lower_f_first)> open(lower_f_first);

g_score[start_idx] = 0.0;
open.push({heuristic(start_col, start_row), start_col, start_row});
```

Inside the neighbor loop, the central A* update has this form:

```cpp
const double tentative_g = g_score[current_idx] + step_cost;
if (tentative_g < g_score[neighbor_idx]) {
  g_score[neighbor_idx] = tentative_g;
  came_from[neighbor_idx] = static_cast<int>(current_idx);
  open.push({tentative_g + heuristic(neighbor_col, neighbor_row),
             neighbor_col, neighbor_row});
}
```

All bounds, obstacle, corner, and closed checks must happen before this update.

Use distances in meters everywhere: Euclidean cell distance multiplied by map
resolution for the heuristic, resolution for an orthogonal step, and
`resolution * sqrt(2)` for a diagonal step. This keeps g and h in matching units.

## Build And Separate Core Bugs From ROS Bugs

From `/workspace`:

```sh
colcon build --symlink-install --packages-select a_star_planner
source install/setup.bash
```

Before blaming A*, verify the wrapper has all three prerequisites:

```sh
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /map --once
ros2 topic echo /goal_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

No `/plan` at all usually means the wrapper is missing the map, goal, or robot TF.
An empty `/plan` after the wrapper receives all three means the endpoints were
blocked/out of bounds or the core found no route.

The returned path should be flat grid indices from start to goal.

## Acceptance Checks

After the planner is wired into its package and Lesson 12 full-stack launch:

```sh
ros2 topic echo /plan
```

In RViz:

- use 2D Goal Pose in free space and confirm a path appears
- put a goal inside an obstacle and confirm an empty path or no valid route
- change `inflation_radius` and observe the path move farther from or closer to walls

Also try start equal to goal; the result should contain that one cell. A path
passing diagonally between two touching obstacles indicates missing corner-cut
prevention. A path that reaches the goal but takes an obvious detour often points
to a priority-queue comparator, move-cost, or g-score update error.

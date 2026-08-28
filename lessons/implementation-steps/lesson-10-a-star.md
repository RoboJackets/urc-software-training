# Project 10 — A* Planning

> **Goal:** Complete the ROS-free A* search that returns grid-cell indices from
> start to goal.

Keep the [Project 10 reference](../reference/project-10-a-star.md) open.

## 1. Parameterize The ROS Wrapper

In `src/a_star_planner/src/a_star_planner.cpp`, replace the three hard-coded
topic strings with declared string parameters:

| Parameter | Default |
| --- | --- |
| `map_topic` | `/map` |
| `goal_topic` | `/goal_pose` |
| `plan_topic` | `/plan` |

Use the [wrapper parameter lines](../reference/project-10-a-star.md#wrapper-parameters)
in the existing subscriptions and publisher. Do not change the map
subscription's transient-local QoS.

## 2. Complete `astarSearch`

Edit:

```text
src/a_star_planner/src/a_star_search.cpp
```

Do not change the declaration in `a_star_search.hpp`. The starter already
validates inputs and creates `g_score`, `came_from`, and `closed`.

Complete the TODOs in order:

1. Write a Euclidean heuristic in meters: cell distance times `grid.resolution`.
2. Create a lowest-f-score-first priority queue and push the start cell.
3. Pop the best cell until the queue is empty. Skip it if already closed;
   otherwise mark it closed.
4. When the goal is popped, follow `came_from` backward and return the reversed
   start-to-goal path.
5. Visit all eight neighboring cells.
6. Reject neighbors that are outside the map, blocked, or closed. For a diagonal
   move, also reject it if either adjacent straight cell is blocked.
7. Use move cost `resolution` for straight moves and
   `resolution * sqrt(2)` for diagonals.
8. Update `g_score`, `came_from`, and the queue only when the new g-score is
   lower.
9. Return an empty vector if the queue empties without reaching the goal.

Use the reference's [queue](../reference/project-10-a-star.md#queue-scaffold),
[search loop](../reference/project-10-a-star.md#search-loop-scaffold),
[relaxation](../reference/project-10-a-star.md#relaxation-shape), and
[reconstruction](../reference/project-10-a-star.md#reconstruction-shape) pieces.

## 3. Build And Check

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select a_star_planner
source install/setup.bash
```

Then follow the Lesson 10 hands-on checkpoint. Send a free-space goal in RViz and
confirm `/plan` contains a path. Also check:

- start equals goal returns one cell
- a blocked goal returns no path
- paths do not cut diagonally through obstacle corners

If no plan message appears, first verify `/map`, `/goal_pose`, and TF
`map -> base_footprint`; those are wrapper inputs, not A* bugs.

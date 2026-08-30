# Lesson 10 — A* Path Planning

> **Goal:** Understand graph search and the A* algorithm, then implement the pure
> search used by `a_star_planner` to find an obstacle-free path and publish it on
> `/plan`.

## Starter files for this lesson

- `src/a_star_planner/src/a_star_search.cpp` — the **pure** A* algorithm (no ROS).
- `src/a_star_planner/src/a_star_planner.cpp` — the ROS wrapper (subscriptions,
  map handling, inflation, path publishing).
- `src/a_star_planner/include/a_star_planner/...` — headers.
- Uses `robonav_training_common`'s `GridMap` (Lesson 8) for world↔grid math.

The "pure algorithm vs. thin ROS wrapper" split (like the particle filter and pure
pursuit) means the search itself is testable without ROS — a recurring repo pattern.

You implement the pure search. Follow
[Lesson 10 implementation steps](implementation-steps/lesson-10-a-star.md) after
reading the algorithm sections below. In a starter repo, the files listed above
should already exist as scaffolding before you begin Lesson 10.

## The planning problem

We now know where the robot is (Lesson 9) and we have a map (Lesson 8). The user
clicks a goal in RViz. **Planning** answers: what's a sequence of waypoints from
here to the goal that stays clear of obstacles and is as short as possible?

We treat the occupancy grid as a **graph**: each free cell is a node; you can move
to neighboring free cells at a cost (the distance). Finding the shortest route
through a graph is a classic search problem.

## From Dijkstra to A*

- **Dijkstra's algorithm** explores outward from the start in order of cost,
  guaranteeing the shortest path — but it expands in *all* directions, wasting work.
- **A\*** adds a **heuristic**: an estimate of the remaining distance to the goal.
  It prioritizes cells that look promising (low cost-so-far *plus* low estimated
  remaining), so it heads toward the goal and explores far less.

A* ranks cells by:

```
f(n) = g(n) + h(n)
```

- **g(n)** — actual cost from the start to cell *n* (known).
- **h(n)** — heuristic estimate of cost from *n* to the goal (a guess).
- **f(n)** — estimated total path cost through *n*.

A* always expands the unvisited cell with the smallest `f`. If the heuristic is
**admissible** (never *overestimates* the true remaining cost), A* is guaranteed to
find the optimal path.

## The heuristic in this repo

Your search should use **Euclidean (straight-line) distance** to the goal as `h`:

```cpp
h = sqrt(dc*dc + dr*dr) * grid.resolution;   // dc, dr = cell deltas to goal
```

Straight-line distance can never be longer than the actual grid path, so it's
admissible → the path is optimal. (Manhattan distance would be inadmissible here
because diagonal moves are allowed.)

## Neighbors, costs, and corner-cutting

The search is **8-connected**: each cell connects to its 4 orthogonal and 4
diagonal neighbors. Move costs scale with real distance:

- orthogonal step: `resolution` (e.g. 0.05 m),
- diagonal step: `resolution · √2` (longer, so it's correctly penalized).

To avoid the robot clipping the corner of an obstacle, a **diagonal move is
forbidden if either of the two orthogonal cells it "cuts past" is blocked**. This
keeps generated paths physically drivable.

## The search, concretely (`astarSearch`)

```cpp
std::vector<int> astarSearch(const GridMap& grid,
                             const std::vector<uint8_t>& blocked,
                             int start_col, int start_row,
                             int goal_col,  int goal_row);
```

Data structures (all flat arrays indexed by cell):

- **`open`** — a `priority_queue` (min-heap) of cells to explore, ordered by `f`.
- **`g_score`** — best known cost-from-start per cell (init ∞).
- **`came_from`** — each cell's predecessor on its best path (for reconstruction).
- **`closed`** — cells already finalized.

The loop:

1. Pop the lowest-`f` cell from `open`. If it's stale (already closed) skip it; mark
   it closed. (The "stale" trick: rather than find-and-update a cell's entry already
   sitting in the heap, we just push a new, better entry and ignore the older one
   when it eventually pops. Simpler code, same result.)
2. If it's the goal, **reconstruct** the path by following `came_from` back to the
   start, and return it (as flat cell indices).
3. Otherwise, for each valid neighbor (in bounds, not blocked, no corner-cut):
   compute tentative `g`; if it beats the neighbor's recorded `g`, update `g_score`
   and `came_from` and push the neighbor with its new `f = g + h`.
4. If `open` empties without reaching the goal → return empty (no path exists).

That's textbook A*, written plainly.

## The ROS wrapper

The node connects A* to the live system.

**Subscribes:**

- **`/map`** (`OccupancyGrid`, transient-local QoS so it gets the latched map) —
  caches the grid and builds the `blocked` mask.
- **`/goal_pose`** (`PoseStamped` — a single pose plus a frame and timestamp) — the
  goal from RViz's **2D Goal Pose** tool (how to use it is in the hands-on below).

**Publishes:** **`/plan`** (`nav_msgs/Path`) — the waypoint list.

**Gets the start pose from TF:** it looks up `map → base_footprint` to find where the
robot currently is (that transform exists thanks to the particle filter + EKF —
Lessons 7 and 9). So planning automatically starts from the robot's localized
position.

### Building the blocked mask + obstacle inflation

The robot is not a point — if the path hugs a wall, the robot's body clips it. So in
`mapCallback`, the planner marks a cell `blocked` if:

- its occupancy ≥ `occupied_threshold` (default 50), or
- it's unknown (-1) and `allow_unknown` is false (default false — treat unknown as
  obstacle, the safe choice), **and then**
- it **inflates**: every blocked cell stamps a disk of blocked cells around it with
  radius `inflation_radius` (default **0.55 m**, converted to cells via the grid
  resolution).

Inflation is what keeps the planned path a safe distance from walls — effectively
"growing" obstacles by the robot's radius so a point-robot search yields a path the
real robot can follow. It's done once per map, keeping the core search simple
(another separation-of-concerns choice).

### Publishing the path (`publishPath`)

The planner converts the path's flat cell indices back to world coordinates (cell
centers, via `GridMap::mapToWorld`) and builds a `nav_msgs/Path` in the `map` frame.
Each pose's **orientation faces the next waypoint** (`atan2` of the direction to the
next point) so the path carries heading information; the last pose keeps the
previous heading. If no path is found, it publishes an **empty** Path to clear any
stale path in RViz and stop the controller.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `occupied_threshold` | 50 | occupancy ≥ this → obstacle |
| `inflation_radius` | 0.55 m | safety buffer grown around obstacles |
| `allow_unknown` | false | if false, unknown cells are obstacles too |
| `map_topic` | `/map` | occupancy-grid input |
| `goal_topic` | `/goal_pose` | requested goal input |
| `plan_topic` | `/plan` | planned-path output |

The topic parameters are a short ROS API repetition: declare them in the
constructor and pass the returned strings into `create_subscription` and
`create_publisher`. Defaults preserve this course's graph, while parameters make
the node reusable without recompiling.

## Hands-on

First complete
[Lesson 10 implementation steps](implementation-steps/lesson-10-a-star.md).

Keep the Lesson 9 localization checkpoint running. After the particle cloud
converges, add the planner in one new sourced terminal:

```sh
ros2 launch a_star_planner a_star_planner.launch.py
```

> **Using the "2D Goal Pose" tool** (your main way to command the robot): it's a
> button in RViz's top toolbar. Click it, then **click-and-drag** on the map — the
> click sets the goal position. RViz includes the drag direction as a goal heading
> in the `/goal_pose` `PoseStamped`, but this training planner intentionally plans
> only to the requested position; path headings follow each path segment.

```sh
ros2 topic echo /plan        # start this before sending a goal; /plan is not latched
```

In RViz:

1. Add a **Path** display on `/plan`, Fixed Frame `map`.
2. After the particle cloud has converged, use **2D Goal Pose** (above) to place a
   goal in free space. A path should appear, snaking around obstacles at a comfortable
   distance (that's inflation).
3. Place a goal *inside* a wall or unreachable region — watch the planner publish an
   empty path (no solution).
4. Tune `inflation_radius` in `a_star_planner.launch.py` and relaunch the planner.
   Too small makes paths hug walls; too large can close narrow passages.

## Video supplement

- [Pathfinding - Understanding A* (Tarodev)](https://www.youtube.com/watch?v=i0x5fj4PqP4)

## Check yourself

- What do g, h, and f represent? Why must h be admissible?
- Why is Euclidean distance admissible here but Manhattan is not?
- What is corner-cutting and how does the search prevent it?
- What is obstacle inflation and why is it done instead of modeling the robot's
  shape inside the search?
- Where does the *start* pose come from, and which earlier lessons make that
  transform available?

## Recap

A* searches the occupancy grid (as an 8-connected graph) for the optimal,
obstacle-free path, ranking cells by `f = g + h` with an admissible Euclidean
heuristic. The ROS wrapper builds a `blocked` mask, **inflates** obstacles by
`inflation_radius` for safety, takes the start from the `map→base_footprint` TF and
the goal from `/goal_pose`, and publishes the result on **`/plan`**. Now we have a
path — the last piece is driving it.

➡️ **Next:** [Lesson 11 — Pure Pursuit Path Following](lesson-11-pure-pursuit.md).

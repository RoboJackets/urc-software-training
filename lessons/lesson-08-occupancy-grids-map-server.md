# Lesson 8 — Occupancy Grids & the Map Server

> **Goal:** Understand how a robot represents a map as an **occupancy grid**, how
> the standard PGM + YAML map format works, the world↔grid coordinate math in
> `robonav_training_common`, and how `map_server` loads and publishes the map that
> the particle filter and planner depend on.

## Where this lives in the repo

- `src/map_server/src/map_server.cpp`, `src/map_server/src/map_loader.cpp` — the
  node and the pure file-loading function.
- `src/map_server/include/map_server/...` — headers.
- `src/map_server/maps/training_map.yaml` (+ `.pgm`) — the actual map.
- `src/robonav_training_common/include/robonav_training_common/occupancy_grid.hpp`
  — the `GridMap` struct and world↔grid conversions used by the planner.

## What is an occupancy grid?

An **occupancy grid** chops the world into a grid of square **cells** and labels
each cell by how likely it is to be occupied by an obstacle. It's the dominant 2D
map representation in mobile robotics because it's simple, supports fast lookups,
and works directly for both localization (does my lidar match?) and planning (is
this cell drivable?).

ROS uses `nav_msgs/OccupancyGrid`, where each cell is an `int8`:

- **0** = free,
- **100** = occupied,
- **-1** = unknown,
- (values 1–99 represent intermediate probabilities; this repo uses just 0/100/-1).

### Grid metadata

A grid is a flat array plus the metadata to place it in the world:

- **`width`, `height`** — cells in x and y.
- **`resolution`** — meters per cell (our map: **0.05 m**, i.e. 5 cm cells).
- **`origin`** — the world `(x, y, yaw)` of the grid's lower-left corner.

The cell array is **row-major**: index `= row * width + col`, with row 0 at the
**bottom**. Remember that — it bites people who think in image coordinates (where
row 0 is the top).

## The standard map format: PGM + YAML

A ROS map is two files:

1. A **`.pgm`** image (grayscale "portable graymap") where each pixel's darkness
   encodes occupancy — dark = wall, light = free.
2. A **`.yaml`** file with the metadata. Open `maps/training_map.yaml`:

```yaml
image: training_map.pgm
resolution: 0.05            # meters per pixel/cell
origin: [-4.5, -4.5, 0.0]   # world x, y, yaw of the lower-left corner
negate: 0
occupied_thresh: 0.65       # ≥ this normalized value → occupied
free_thresh: 0.25           # ≤ this → free; in between → unknown
```

### Pixel → occupancy conversion

`map_loader.cpp` reads the PGM and converts each pixel to a `0/100/-1` cell:

1. Normalize: `occ = (negate ? p : (maxval − p)) / maxval`. With `negate: 0`,
   **darker pixels mean higher occupancy** (the usual convention).
2. Threshold:
   - `occ > occupied_thresh` (0.65) → **100** (occupied),
   - `occ < free_thresh` (0.25) → **0** (free),
   - otherwise → **-1** (unknown).
3. Flip rows: PGM row 0 is the top of the image, but OccupancyGrid row 0 is the
   bottom, so the loader maps `img_row = height − 1 − row`.

This is why a hand-drawn or SLAM-generated PGM, plus a little YAML, becomes a map
your code can use.

## The `GridMap` helper (shared math)

Planning needs to convert between **world coordinates** (meters, what TF and sensors
use) and **grid indices** (col/row, what the array uses). That reusable math lives in
`occupancy_grid.hpp` as the `GridMap` struct. The map server only loads cells and
metadata, so it does not need these conversions itself:

```cpp
struct GridMap {
  int    width, height;
  double resolution;
  double origin_x, origin_y;

  bool worldToMap(double wx, double wy, int& col, int& row) const;  // meters → cell
  void mapToWorld(int col, int row, double& wx, double& wy) const;  // cell → meters (center)
  bool inBounds(int col, int row) const;
  std::size_t index(int col, int row) const;   // row * width + col
  std::size_t cellCount() const;               // width * height
};
```

The key formulas:

- **world → grid:** `col = floor((wx − origin_x) / resolution)` (and likewise row).
  Returns `false` if the point falls outside the grid.
- **grid → world (cell center):** `wx = origin_x + (col + 0.5) · resolution`. The
  `+ 0.5` returns the *center* of the cell, not its corner — important so paths run
  through cell centers.

Putting this in one shared, header-only helper (no compilation, just include it)
keeps later map consumers from each inventing their own conversion rules.

## The `map_server` node

`map_server.cpp` is intentionally tiny — it separates file parsing
(`loadMapFromYaml()` in `src/map_server/src/map_loader.cpp`, which returns a
`nav_msgs/OccupancyGrid` and throws `std::runtime_error` on failure) from the live
ROS wrapper (the `MapServer` node). That split makes the parser testable without
spinning up a node or ROS graph. Use the loader behavior described here to check YAML
metadata, thresholds, row orientation, and both ASCII and 16-bit binary PGM data.

The node:

1. Declares a parameter `map_yaml_path` (the absolute path to the YAML).
2. Calls `loadMapFromYaml()` to build the grid (frame_id `map`).
3. Publishes it **once** on **`/map`** (`nav_msgs/OccupancyGrid`).

### Latching with transient-local QoS

First, the term. **QoS** ("Quality of Service") is a set of options publishers and
subscribers agree on for *how* messages are delivered — reliability (guaranteed vs.
best-effort), history depth, and **durability** (whether the last message is kept
for late joiners). A subscriber must use a *compatible* QoS to receive a topic.

The map never changes, so re-publishing it 10×/second would be wasteful. Instead the
publisher uses **transient-local** durability (plus reliable, depth 1) — meaning
**ROS keeps the last message so any subscriber that connects *later* still receives
it immediately**. This is exactly what older ROS called a "**latched**" topic, and
it's the standard pattern for static, set-once data. A planner started a minute after
the map server still gets the map. (Both the particle filter and planner explicitly
request reliable, transient-local QoS, so they receive it.)

## Who uses the map

- **`particle_filter`** (Lesson 9) builds its likelihood field from `/map` and
  scatters initial particles over free cells.
- **`a_star_planner`** (Lesson 10) uses `/map` (via `GridMap`) to know which cells
  are drivable and to inflate obstacles.

And critically (Lesson 6): the obstacles in `training_world.sdf` are arranged to
**match** this map, so the lidar sees the same walls the map records.

## Hands-on

No coding changes are assigned for the map server in this course. Treat it as a
provided dependency for localization and planning.

First build the map server and inspect the map files it will load:

```sh
colcon build --packages-up-to map_server
source install/setup.bash
less src/map_server/maps/training_map.yaml
```

There is no automated parser test suite in this training repository. The runtime
check below verifies that the loader accepts the supplied PGM/YAML pair and
publishes the expected metadata.

Then launch and inspect in two sourced terminals:

```sh
# Terminal 1: the launch file supplies the default map path
ros2 launch map_server map_server.launch.py
```

```sh
# Terminal 2
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /map --once
ros2 topic info /map --verbose   # Durability should be TRANSIENT_LOCAL
```

In RViz:

1. Add a **Map** display on `/map`, Fixed Frame `map`. You'll see the occupancy
   grid as black (occupied), white/gray (free), and unknown regions.
2. Read off the `info` from the `ros2 topic echo --qos-reliability reliable
   --qos-durability transient_local /map --once` command: confirm `resolution: 0.05` and
   `origin: [-4.5, -4.5, 0]` match the YAML.
3. (Optional) Open `training_map.yaml`; change `free_thresh`/`occupied_thresh`,
   then rebuild and relaunch the map server. The YAML is installed with the package,
   so a rebuild is the reliable rule for this map-file edit.

## Check yourself

- What do 0, 100, and -1 mean in an `OccupancyGrid`?
- Given `origin = (-4.5, -4.5)` and `resolution = 0.05`, what cell (col,row) is the
  world point (0, 0)? What world point is the *center* of cell (90, 90)?
- Why does `mapToWorld` add 0.5 to col/row?
- Why does `map_server` use transient-local QoS instead of republishing?
- Why is the file-parsing logic split out from the ROS node?

## Recap

A map is an **occupancy grid**: cells of `resolution` meters labeled free (0),
occupied (100), or unknown (-1), placed in the world by an **origin**. ROS stores
maps as **PGM + YAML**, which `map_loader.cpp` converts (with thresholds and a row
flip) into a `nav_msgs/OccupancyGrid`. `map_server` publishes it once on **`/map`**
with **latched QoS**. The shared **`GridMap`** helper gives the planner consistent
world↔grid conversions. Now we can localize against this map.

➡️ **Next:** [Lesson 9 — Particle Filter (MCL)](lesson-09-particle-filter-mcl.md).

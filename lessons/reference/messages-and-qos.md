# Messages And QoS

ROS topics carry one message type. The type tells you which fields exist and which
package dependency you need.

Inspect any message:

```sh
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show sensor_msgs/msg/LaserScan
```

## Include Syntax

ROS message type names use `/` at the command line and `::` in C++.

| Command-line type | C++ include | C++ type |
| --- | --- | --- |
| `std_msgs/msg/String` | `<std_msgs/msg/string.hpp>` | `std_msgs::msg::String` |
| `geometry_msgs/msg/Twist` | `<geometry_msgs/msg/twist.hpp>` | `geometry_msgs::msg::Twist` |
| `geometry_msgs/msg/PoseStamped` | `<geometry_msgs/msg/pose_stamped.hpp>` | `geometry_msgs::msg::PoseStamped` |
| `geometry_msgs/msg/PoseArray` | `<geometry_msgs/msg/pose_array.hpp>` | `geometry_msgs::msg::PoseArray` |
| `geometry_msgs/msg/PoseWithCovarianceStamped` | `<geometry_msgs/msg/pose_with_covariance_stamped.hpp>` | `geometry_msgs::msg::PoseWithCovarianceStamped` |
| `geometry_msgs/msg/TransformStamped` | `<geometry_msgs/msg/transform_stamped.hpp>` | `geometry_msgs::msg::TransformStamped` |
| `nav_msgs/msg/Odometry` | `<nav_msgs/msg/odometry.hpp>` | `nav_msgs::msg::Odometry` |
| `nav_msgs/msg/OccupancyGrid` | `<nav_msgs/msg/occupancy_grid.hpp>` | `nav_msgs::msg::OccupancyGrid` |
| `nav_msgs/msg/Path` | `<nav_msgs/msg/path.hpp>` | `nav_msgs::msg::Path` |
| `sensor_msgs/msg/JointState` | `<sensor_msgs/msg/joint_state.hpp>` | `sensor_msgs::msg::JointState` |
| `sensor_msgs/msg/LaserScan` | `<sensor_msgs/msg/laser_scan.hpp>` | `sensor_msgs::msg::LaserScan` |
| `sensor_msgs/msg/Imu` | `<sensor_msgs/msg/imu.hpp>` | `sensor_msgs::msg::Imu` |

## Message Types Used In This Repo

| Topic | Type | Used by |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | pure pursuit publishes, diff drive controller consumes |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz publishes, A* subscribes |
| `/joint_states` | `sensor_msgs/msg/JointState` | wheel odometry subscribes |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | wheel odometry publishes, EKF subscribes |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF publishes, particle filter subscribes |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | particle filter subscribes |
| `/imu/data` | `sensor_msgs/msg/Imu` | EKF subscribes |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map server publishes, planner/PF subscribe |
| `/plan` | `nav_msgs/msg/Path` | A* publishes, pure pursuit subscribes |
| `/particle_cloud` | `geometry_msgs/msg/PoseArray` | particle filter publishes |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | particle filter publishes |

## Common Field Patterns

### `geometry_msgs::msg::Twist`

```cpp
geometry_msgs::msg::Twist cmd;
cmd.linear.x = 0.3;    // forward speed, m/s
cmd.angular.z = 1.0;   // yaw rate, rad/s
```

For a planar diff-drive robot, most code only uses `linear.x` and `angular.z`.

### `geometry_msgs::msg::PoseStamped`

```cpp
geometry_msgs::msg::PoseStamped pose;
pose.header.frame_id = "map";
pose.header.stamp = now();
pose.pose.position.x = x;
pose.pose.position.y = y;
pose.pose.position.z = 0.0;
pose.pose.orientation = quaternion_msg;
```

`header.frame_id` tells you which coordinate frame the pose is expressed in.

### `nav_msgs::msg::Odometry`

```cpp
nav_msgs::msg::Odometry odom;
odom.header.frame_id = "odom";
odom.child_frame_id = "base_footprint";
odom.pose.pose.position.x = x;
odom.pose.pose.position.y = y;
odom.twist.twist.linear.x = vx;
odom.twist.twist.angular.z = wz;
```

`header.frame_id` is the parent frame. `child_frame_id` is the moving body frame.

Convert a message timestamp and calculate elapsed seconds with:

```cpp
const rclcpp::Time stamp(msg->header.stamp);
const double dt = (stamp - previous_stamp).seconds();
if (dt <= 0.0) {
  return;
}
```

For planar covariance, the diagonal entries for x, y, and yaw are indices 0, 7,
and 35 in the flat 6-by-6 array.

### `nav_msgs::msg::Path`

```cpp
nav_msgs::msg::Path path;
path.header.frame_id = "map";
path.header.stamp = now();

geometry_msgs::msg::PoseStamped waypoint;
waypoint.header = path.header;
waypoint.pose.position.x = wx;
waypoint.pose.position.y = wy;

path.poses.push_back(waypoint);
```

### `nav_msgs::msg::OccupancyGrid`

```cpp
const int width = static_cast<int>(msg->info.width);
const int height = static_cast<int>(msg->info.height);
const double resolution = msg->info.resolution;
const double origin_x = msg->info.origin.position.x;
const double origin_y = msg->info.origin.position.y;

const int index = row * width + col;
const int8_t value = msg->data[index];
```

Cell values:

- `0` means free.
- `100` means occupied.
- `-1` means unknown.

### `sensor_msgs::msg::JointState`

```cpp
static int findJointIndex(
  const sensor_msgs::msg::JointState & msg, const std::string & joint_name)
{
  for (std::size_t index = 0; index < msg.name.size(); ++index) {
    if (msg.name[index] == joint_name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}
```

Do not assume joint order. Search `name`, reject `-1`, and verify the matching
index is smaller than `position.size()` before reading `position[index]`.

### `sensor_msgs::msg::LaserScan`

```cpp
for (size_t i = 0; i < msg->ranges.size(); ++i) {
  const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
  const double range = msg->ranges[i];
  if (!std::isfinite(range)) {
    continue;
  }
}
```

Skip invalid ranges and max-range readings when they do not help your model.

### `sensor_msgs::msg::Imu`

Important fields:

- `orientation`: roll/pitch/yaw as a quaternion
- `angular_velocity`: rad/s
- `linear_acceleration`: m/s^2
- covariance arrays: uncertainty for each measurement group

In this repo, the EKF config decides which IMU fields to fuse.

## Quaternion Helpers

Yaw to quaternion:

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

tf2::Quaternion q;
q.setRPY(0.0, 0.0, yaw);
msg.pose.orientation = tf2::toMsg(q);
```

Quaternion to yaw:

```cpp
#include <tf2/utils.h>

tf2::Quaternion q(
  msg->pose.pose.orientation.x,
  msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z,
  msg->pose.pose.orientation.w);
const double yaw = tf2::getYaw(q);
```

## QoS Basics

QoS controls how publishers and subscribers exchange messages. Publisher and
subscriber QoS must be compatible.

### Default Queue Depth

Use this for normal live topics where old messages are not useful.

```cpp
pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

sub_ = create_subscription<nav_msgs::msg::Path>(
  "/plan", 10,
  std::bind(&MyNode::pathCallback, this, std::placeholders::_1));
```

Good for:

- `/cmd_vel`
- `/goal_pose`
- `/plan`
- `/joint_states`
- odometry topics

### Sensor Data QoS

Use this for high-rate sensor streams when dropping an old sample is better than
blocking on reliability.

```cpp
scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
  "/lidar/scan", rclcpp::SensorDataQoS(),
  std::bind(&MyNode::scanCallback, this, std::placeholders::_1));
```

Good for:

- `/lidar/scan`
- `/imu/data`

### Transient-Local QoS

Use this for latched/static data. The publisher keeps the last message so late
subscribers can receive it.

Publisher:

```cpp
rclcpp::QoS map_qos(1);
map_qos.transient_local().reliable();

map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
```

Subscriber:

```cpp
rclcpp::QoS map_qos(1);
map_qos.transient_local().reliable();

map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", map_qos,
  std::bind(&MyNode::mapCallback, this, std::placeholders::_1));
```

Good for:

- `/map`

Answer-key or later-starter examples:

- Publisher example: `src/map_server/src/map_server.cpp` (provided map server)
- Subscriber example: `src/a_star_planner/src/a_star_planner.cpp` once the planner
  starter exists

## QoS Debug Commands

```sh
ros2 topic info /map --verbose
ros2 topic info /lidar/scan --verbose
ros2 topic hz /lidar/scan
```

If a topic exists but your callback never runs, check QoS compatibility.

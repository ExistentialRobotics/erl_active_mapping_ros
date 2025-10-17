# `erl_active_mapping_ros`

[![Tags](https://img.shields.io/github/v/tag/ExistentialRobotics/erl_active_mapping_ros?label=version)](https://github.com/ExistentialRobotics/erl_active_mapping_ros/tags)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-jazzy-blue)](https://docs.ros.org/)
<!-- [![ROS1](https://img.shields.io/badge/ROS1-noetic-blue)](http://wiki.ros.org/) -->

This package provides ROS nodes that wrap the `erl_active_mapping` library for autonomous exploration and active mapping in robotics applications. It includes support for frontier-based exploration strategies with 2D grid mapping.

![](assets/demo_frontier_grid_2d_ros_10x.gif)

## Features

- **Frontier-Based Exploration**: Autonomous exploration using frontier detection
- **Online Mapping**: Real-time occupancy map building from laser scan data
- **External Map Support**: Option to use pre-built maps for exploration planning
- **ROS2 Integration**: Full integration with ROS2 interfaces
- **Flexible Precision**: Supports both single and double precision computation
- **Configurable QoS**: Flexible QoS settings for all topics
- **Auto-Replanning**: Automatic path replanning based on exploration progress
- **Exploration Metrics**: Real-time tracking of distance traveled, observed area, and exploration ratio

## Nodes

### Common Parameters for Active Mapping Nodes

All active mapping nodes inherit from a common base class (`ActiveMappingNode`) that provides the following shared parameters:

#### Core Settings
- `double_precision` (bool, default: false): Use double precision instead of float
- `global_frame` (string, default: "map"): Global reference frame
- `robot_frame` (string, default: "base_link"): Robot reference frame

#### Exploration Control
- `auto_replan` (bool, default: false): Enable automatic path replanning
- `stop_exploration_ratio` (double, default: 0.95): Stop exploration when this ratio of area is observed (0.0-1.0)
- `max_observed_area` (double, default: 10.0): Maximum expected observable area for computing exploration ratio

#### Default QoS Settings
- `default_qos_reliability` (string, default: "reliable"): Default QoS reliability for topics
- `default_qos_durability` (string, default: "volatile"): Default QoS durability for topics

#### Output Topics
- `path_topic` (string, default: "path"): Planned path output topic
- `path_topic_reliability` (string, default: uses default_qos_reliability)
- `path_topic_durability` (string, default: uses default_qos_durability)
- `dist_topic` (string, default: "distance"): Total distance traveled output topic
- `dist_topic_reliability` (string, default: uses default_qos_reliability)
- `dist_topic_durability` (string, default: uses default_qos_durability)
- `observed_area_topic` (string, default: "observed_area"): Observed area output topic
- `observed_area_topic_reliability` (string, default: uses default_qos_reliability)
- `observed_area_topic_durability` (string, default: uses default_qos_durability)
- `observed_ratio_topic` (string, default: "observed_ratio"): Exploration ratio output topic
- `observed_ratio_topic_reliability` (string, default: uses default_qos_reliability)
- `observed_ratio_topic_durability` (string, default: uses default_qos_durability)
- `replan_topic` (string, default: "replan"): Replan signal output topic
- `replan_topic_reliability` (string, default: uses default_qos_reliability)
- `replan_topic_durability` (string, default: uses default_qos_durability)

#### Services
- `plan_srv_name` (string, default: "plan_path"): Planning service name

---

### 1. frontier_based_grid_2d_node

Frontier-based autonomous exploration planner for 2D grid-based environments.

#### Subscriptions

- `map` (nav_msgs/OccupancyGrid): External occupancy grid map (when `use_external_map` is true, default topic: "map")
- `scan` (sensor_msgs/LaserScan): Laser scan data for online mapping (when `use_external_map` is false, default topic: "scan")

#### Publications

- `path` (nav_msgs/Path): Planned exploration path
- `distance` (std_msgs/Float64): Total distance traveled by the robot
- `observed_area` (std_msgs/Float64): Total observed area in square meters
- `observed_ratio` (std_msgs/Float64): Ratio of observed area to maximum expected area (0.0-1.0)
- `replan` (std_msgs/Bool): Signal indicating whether replanning is needed
- `internal_map` (nav_msgs/OccupancyGrid): Internal occupancy map (default topic: "internal_map")

#### Services

- `plan_path` (std_srvs/srv/Trigger): Trigger exploration path planning

#### Parameters

See [Common Parameters for Active Mapping Nodes](#common-parameters-for-active-mapping-nodes) above.

**Additional Node-Specific Parameters:**

*Agent Configuration:*
- `agent_config_file` (string, **required**): Path to agent configuration YAML file

*Map Configuration (for online mapping):*
- `map_min` (double[], default: [-10.1, -10.1]): Minimum corner of the map [x_min, y_min]
- `map_max` (double[], default: [10.1, 10.1]): Maximum corner of the map [x_max, y_max]
- `map_resolution` (double, default: 0.05): Map resolution in meters per cell

*Map Source:*
- `use_external_map` (bool, default: false): Use external map (true) or build map from laser scans (false)

*Input Topics (when use_external_map is true):*
- `map_topic` (string, default: "map"): External occupancy grid topic
- `map_topic_reliability` (string, default: "reliable"): QoS reliability setting
- `map_topic_durability` (string, default: "transient_local"): QoS durability setting

*Input Topics (when use_external_map is false):*
- `scan_topic` (string, default: "scan"): Laser scan topic
- `scan_topic_reliability` (string, default: "reliable"): QoS reliability setting
- `scan_topic_durability` (string, default: "volatile"): QoS durability setting

*Output Topics:*
- `internal_map_topic` (string, default: "internal_map"): Internal map topic
- `internal_map_topic_reliability` (string, default: "reliable"): QoS reliability setting
- `internal_map_topic_durability` (string, default: "transient_local"): QoS durability setting

## Usage Examples

### Basic Frontier-Based Exploration with Online Mapping

```bash
# Launch with default parameters (online mapping from laser scans)
ros2 run erl_active_mapping_ros frontier_based_grid_2d_node --ros-args \
  -p agent_config_file:=/path/to/frontier_based_grid_2d.yaml \
  -p auto_replan:=true \
  -p stop_exploration_ratio:=0.95 \
  -p map_min:="[-10.0, -10.0]" \
  -p map_max:="[10.0, 10.0]" \
  -p map_resolution:=0.05 \
  -p max_observed_area:=100.0

# Trigger initial planning if auto_replan is false
ros2 service call /plan_path std_srvs/srv/Trigger
```

### Exploration with External Map

```bash
# Launch with external map
ros2 run erl_active_mapping_ros frontier_based_grid_2d_node --ros-args \
  -p agent_config_file:=/path/to/frontier_based_grid_2d.yaml \
  -p use_external_map:=true \
  -p auto_replan:=true \
  -p stop_exploration_ratio:=0.95 \
  -p max_observed_area:=400.0

# The node will subscribe to /map and use it for planning
```

### Custom Parameters

```bash
# Launch with custom parameters
ros2 run erl_active_mapping_ros frontier_based_grid_2d_node --ros-args \
  -p double_precision:=true \
  -p global_frame:=map \
  -p robot_frame:=base_link \
  -p auto_replan:=true \
  -p stop_exploration_ratio:=0.90 \
  -p agent_config_file:=/path/to/config.yaml \
  -p map_resolution:=0.1 \
  -p scan_topic:=scan \
  -p path_topic:=exploration_path \
  -p default_qos_reliability:=reliable \
  -p default_qos_durability:=transient_local
```

### Using Launch File

```bash
# Launch test example
ros2 launch erl_active_mapping_ros test_frontier_based_grid_2d_node_launch.py

# Launch with rqt_plot for monitoring
ros2 launch erl_active_mapping_ros test_frontier_based_grid_2d_node_launch.py open_rqt_plot:=true
```

### Monitoring Exploration Progress

```bash
# Monitor distance traveled
ros2 topic echo /distance

# Monitor observed area
ros2 topic echo /observed_area

# Monitor exploration ratio
ros2 topic echo /observed_ratio

# Check if replanning is needed
ros2 topic echo /replan
```

## Agent Configuration

The `agent_config_file` parameter should point to a YAML file that configures the frontier-based exploration agent. Example configuration can be found in the `erl_active_mapping` package under `config/frontier_based_grid_2d.yaml`.

Key configuration parameters typically include:
- Frontier detection thresholds
- Path planning parameters
- Sensor model parameters
- Exploration strategy settings

## Dependencies

- `erl_active_mapping`: Core active mapping algorithms
- `erl_env`: Environment representations
- `erl_common`: Common utilities
- `nav_msgs`: Navigation messages for paths and maps
- `sensor_msgs`: Sensor messages for laser scans
- `geometry_msgs`: Geometry messages for poses
- `std_msgs`: Standard messages
- `std_srvs`: Standard services
- `tf2_ros`: Transform library for ROS2
- ROS2 Humble or later

## Building

### ROS2

```bash
# In your ROS2 workspace
colcon build --packages-select erl_active_mapping_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

## Topics Summary

### Published Topics

| Topic             | Type                   | Description                  |
| ----------------- | ---------------------- | ---------------------------- |
| `/path`           | nav_msgs/Path          | Planned exploration path     |
| `/distance`       | std_msgs/Float64       | Total distance traveled      |
| `/observed_area`  | std_msgs/Float64       | Total observed area (m²)     |
| `/observed_ratio` | std_msgs/Float64       | Exploration completion ratio |
| `/replan`         | std_msgs/Bool          | Replanning needed signal     |
| `/internal_map`   | nav_msgs/OccupancyGrid | Internal occupancy map       |

### Subscribed Topics

| Topic   | Type                   | Description             | Condition                |
| ------- | ---------------------- | ----------------------- | ------------------------ |
| `/map`  | nav_msgs/OccupancyGrid | External occupancy grid | `use_external_map=true`  |
| `/scan` | sensor_msgs/LaserScan  | Laser scan data         | `use_external_map=false` |

### Services

| Service      | Type                 | Description           |
| ------------ | -------------------- | --------------------- |
| `/plan_path` | std_srvs/srv/Trigger | Trigger path planning |

## License

This package is released under the MIT License. See the [LICENSE](LICENSE) file for details.

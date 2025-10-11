# aruco_id_checker

This ROS 2 package searches for a specific Aruco marker in a marker array and publishes its pose.

## Features

- Subscribes to the `aruco/markers` topic (`aruco_markers_msgs/MarkerArray`)
- Checks if a marker with a given ID is present
- Publishes the pose of the target marker to `/target_pose` (`geometry_msgs/PoseStamped`)
- Configurable target ID via launch file or parameter

## Installation

1. Clone this repository into your ROS 2 workspace `src` directory.
2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the package:
   ```bash
   colcon build --packages-select aruco_id_checker
   ```
4. Source the workspace overlay:
   ```bash
   source install/setup.bash
   ```

## Usage

Launch the node with the launch file:

```bash
ros2 launch aruco_id_checker pose_publisher.launch.py
```

By default, the node searches for marker ID `7`. You can change the target ID in the launch file:

```python
parameters=[{'target_id': 42}]
```

## Node

### `pose_publisher`

- **Parameters**
  - `target_id` (int, default: 0): The Aruco ID to search for
- **Subscribed topics:** `aruco/markers` (`aruco_markers_msgs/MarkerArray`)
- **Published topics:** `/target_pose` (`geometry_msgs/PoseStamped`)

## Testing

To run linter and style tests:

```bash
colcon test --packages-select aruco_id_checker
```

## 📄 License

This project is licensed under the BSD-3-Clause License.

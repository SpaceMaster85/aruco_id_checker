# aruco_id_checker

**Publishes a `PoseStamped` message for a selected Aruco marker ID from `aruco_markers_msgs`.**

---

## 📦 Package Overview

- **Node:** `pose_publisher`  
- **Input Topic:** `aruco_markers` (`aruco_markers_msgs/MarkerArray`)  
- **Output Topic:** `/target_pose` (`geometry_msgs/PoseStamped`)  
- **Parameter:**  
  - `target_id` (int) – The Aruco ID for which the pose should be published.

- Uses ROS2 package: [`aruco_markers_msgs`](https://github.com/SpaceMaster85/aruco_markers_msgs)

---

## 🛠 Installation

### 1. Create Workspace (if not already exists)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone Dependencies

```bash
git clone https://github.com/SpaceMaster85/aruco_markers_msgs.git
```

### 3. Add Package

Place `aruco_id_checker` in `src/` and add all files:

```
aruco_id_checker/
├── package.xml
├── setup.py
├── resource/
├── launch/
└── aruco_id_checker/
```

### 4. Build Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### Run Node Directly

```bash
ros2 run aruco_id_checker pose_publisher --ros-args -p target_id:=7
```

### Run Node with Launch File

```bash
ros2 launch aruco_id_checker pose_publisher.launch.py
```

`target_id` can be overridden at launch:

```bash
ros2 launch aruco_id_checker pose_publisher.launch.py target_id:=5
```

---

## 🔍 Verification

### Show Available Topics

```bash
ros2 topic list
```

### Receive the Pose

```bash
ros2 topic echo /target_pose
```

The topic `/aruco_markers` must be published by a marker publisher (e.g., camera node).

---

## ⚙️ Package Structure

```
aruco_pose_publisher/
├── package.xml
├── setup.py
├── resource/
│   └── aruco_pose_publisher
├── launch/
│   └── pose_publisher.launch.py
└── aruco_pose_publisher/
    ├── __init__.py
    └── pose_publisher_node.py
```

---

## 💡 Notes

- The package publishes the pose only for the first marker that matches the `target_id`.

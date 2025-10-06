# Double Ackerman Steering Stack for Agriculture Robot

[![ROS 2](https://img.shields.io/badge/ROS-2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 📋 Overview

This repository contains a complete ROS 2 stack for an autonomous agricultural robot with **double Ackerman steering** (four-wheel independent steering). The system is designed for autonomous navigation in agricultural environments, specifically tailored for row-crop operations such as maize field navigation.

### Key Features

- **Double Ackerman Steering**: Four-wheel independent steering mechanism providing superior maneuverability
- **Autonomous Navigation**: Full Nav2 integration with custom controllers optimized for agricultural environments
- **3D SLAM**: Real-time mapping using RTAB-Map with stereo camera support
- **Agricultural Environment Simulation**: Procedurally generated maize field environments using Gazebo
- **Advanced Path Planning**: Multiple controller options including Dynamic Window Pure Pursuit and Vector Pursuit
- **Obstacle Detection**: Stereo vision-based obstacle detection and avoidance
- **Teleoperation**: Manual control capability via joystick interface

---

## 🏗️ System Architecture

### Robot Configuration
- **Steering Type**: Double Ackerman (4-wheel independent steering)
- **Dimensions**:
  - Wheelbase: 2.39 m
  - Track Width: 1.34 m
  - Wheel Radius: 0.4 m
- **Mass**: ~464 kg (base) + additional components
- **Sensors**:
  - 2x Stereo RGB-D Cameras (left and right)
  - Joint state sensors for all steering and drive joints
  - IMU (integrated in sensor suite)

### Main Components

```
double_ackerman_steering_stack/
├── src/
│   ├── autonomous/                          # Autonomous navigation packages
│   │   ├── agriculture_robot_navigation/    # Navigation configuration and launch files
│   │   ├── agriculture_robot_viz/           # RViz visualization configurations
│   │   ├── control/                         # Controller implementations
│   │   │   ├── nav2_dynamic_window_pure_pursuit_controller/
│   │   │   ├── vector_pursuit_controller/
│   │   │   ├── teb_local_planner/
│   │   │   └── costmap_converter/
│   │   ├── rtabmap/                         # RTAB-Map core library
│   │   └── rtabmap_ros/                     # RTAB-Map ROS 2 integration
│   ├── Robot_urdf/                          # Robot description and simulation
│   ├── Simulation_Agriculture_Field_Generator/ # Virtual maize field generator
│   └── teleop_twist_joy/                    # Joystick teleoperation
├── build/                                   # Build artifacts
├── install/                                 # Installation files
└── log/                                     # Build and runtime logs
```

---

## 📦 Package Details

### 1. **agriculture_robot_navigation** (v1.0.5)
Core navigation package providing Nav2 integration for agricultural robots.

**Features:**
- Custom Nav2 parameter configurations optimized for agricultural environments
- SLAM integration (both synchronous and asynchronous modes)
- Support for RGBD scan processing
- Autonomous operation launch files
- Behavior tree configurations for navigation tasks

**Key Files:**
- `config/nav2.yaml`: Nav2 parameter configuration
- `config/slam.yaml`: SLAM Toolbox configuration
- `launch/agri_robot_autonomous_operation.launch.py`: Main autonomous navigation launch
- `launch/slam_async.launch.py`: Asynchronous SLAM launch
- `launch/agri_robot_rgbd_scan.launch.py`: RGBD sensor processing

### 2. **Robot_urdf**
Complete robot description package including URDF models, Gazebo plugins, and control interfaces.

**Features:**
- Detailed URDF model with 8 joints (4 steering + 4 drive)
- Stereo camera integration (left and right)
- Gazebo simulation support with Ackerman steering plugin
- Custom rear steering commander for double Ackerman coordination
- Physics-accurate inertial and collision models

**Key Components:**
- **Steering Joints**: 
  - `steer_front_left_link_j`, `steer_front_right_link_j`
  - `steer_back_left_link_j`, `steer_back_right_link_j`
- **Drive Joints**:
  - `drive_front_left_link_j`, `drive_front_right_link_j`
  - `drive_back_left_link_j`, `drive_back_right_link_j`
- **Sensors**:
  - Left and right RGB-D cameras with depth sensing
  - Joint state publishers for all actuated joints

**Key Files:**
- `urdf/Robot_urdf.urdf`: Main robot description
- `launch/launch.py`: Robot state publisher and visualization
- `launch/gz_simulator_launch.py`: Gazebo simulation launcher
- `config/double_ackerman_params.yaml`: Vehicle parameters

### 3. **nav2_dynamic_window_pure_pursuit_controller**
Custom Nav2 controller plugin combining Dynamic Window Approach (DWA) with Pure Pursuit.

**Features:**
- Dynamic velocity and acceleration control
- Adaptive lookahead distance
- Collision detection and avoidance
- Optimized for agricultural row navigation
- Configurable velocity scaling

**Parameters:**
- Desired linear velocity: 3.0 m/s
- Desired angular velocity: 0.2 rad/s
- Max linear acceleration: 1.25 m/s²
- Max angular acceleration: 1.05 rad/s²
- Lookahead distance: 0.3-2.0 m (adaptive)

### 4. **vector_pursuit_controller** (v1.1.0)
Alternative controller using vector field-based path following.

**Advantages:**
- Smooth trajectory following
- Better suited for structured environments
- Reduced oscillations in narrow spaces

### 5. **Simulation_Agriculture_Field_Generator** (Virtual Maize Field)
Procedural generation of realistic agricultural environments for testing and validation.

**Features:**
- Randomized maize field generation
- Configurable row spacing and plant density
- Gazebo Classic integration
- Realistic plant models and terrain

**Usage:**
```bash
ros2 launch virtual_maize_field generate_world.launch.py
```

### 6. **rtabmap** and **rtabmap_ros**
Real-Time Appearance-Based Mapping for 3D SLAM using stereo cameras.

**Features:**
- RGB-D SLAM with loop closure detection
- 3D point cloud generation
- Occupancy grid mapping
- Visual odometry
- Multi-session mapping support

**Modes:**
- Asynchronous SLAM (high-speed operation)
- Synchronous SLAM (precise mapping)

### 7. **teb_local_planner**
Time-Elastic Band local planner with Ackerman steering support.

**Features:**
- Support for car-like robots
- Dynamic obstacle avoidance
- Trajectory optimization
- Cmd_vel to Ackerman drive conversion script

### 8. **teleop_twist_joy**
Joystick-based teleoperation interface.

**Features:**
- Manual control override
- Safety deadman switch
- Velocity scaling
- Compatible with standard gaming controllers

---

## 🚀 Installation

### Prerequisites

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Gazebo Classic 11
- **Python**: 3.10+

### Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install required ROS 2 packages
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-tf2-tools \
  ros-humble-rtabmap-ros
```

### Build Instructions

1. **Clone the repository:**
```bash
cd ~
git clone https://github.com/ebrahimabdelghfar/double_ackerman_steering_stack.git
cd double_ackerman_steering_stack
```

2. **Install workspace dependencies:**
```bash
cd ~/double_ackerman_steering_stack
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**
```bash
colcon build --symlink-install
```

4. **Source the workspace:**
```bash
source install/setup.bash
```

---

## 🎮 Usage

### 2. Simulation with Gazebo

Launch the robot in a simulated agricultural environment:

```bash
# Terminal 1: Launch Gazebo with the robot and maize field it will run headless
ros2 launch Robot_urdf gz_simulator_launch.py

# Terminal 2: Launch RTAB-Map grid map visulaizations and odometry
ros2 launch agriculture_robot_navigation agri_robot_rgbd_scan_demo.launch.py localization:=true

# Terminal 3: Launch navigation stack
ros2 launch agriculture_robot_navigation agri_robot_autonomous_operation.launch.py

```

### 3. Send Navigation Goals

#### Using RViz
1. Open RViz with the navigation configuration
2. Click "2D Nav Goal" button
3. Click and drag on the map to set goal pose

#### Using Command Line
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, 
    pose: {position: {x: 5.0, y: 0.0, z: 0.0}, 
           orientation: {w: 1.0}}}"
```

#### Using Python API
```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 0.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)
```

### 3. Manual Teleoperation

```bash
# Launch teleoperation node
ros2 launch teleop_twist_joy teleop-launch.py

# Use joystick controls:
# - Left stick: Linear velocity
# - Right stick: Angular velocity
# - Hold deadman button (typically L1/LB) to enable movement
```

### 4. SLAM Mapping

#### Create a new map:
```bash
# Synchronous SLAM (more accurate)
ros2 launch agriculture_robot_navigation agri_robot_rgbd_scan_demo.launch.py localization:=false
```
### 5. Custom World Generation

Generate custom agricultural environments:

```bash
ros2 run virtual_maize_field generate_world.py \
  --row_length 10.0 \
  --rows 12 \
  --row_spacing 0.75 \
  --plant_spacing 0.13 \
```

---

## ⚙️ Configuration

### Navigation Parameters

Edit `src/autonomous/agriculture_robot_navigation/config/nav2.yaml` to customize:

- **Controller frequency**: 30 Hz (adjust for real-time performance)
- **Velocity limits**: Linear 3.0 m/s, Angular 0.2 rad/s
- **Lookahead distance**: 0.3-2.0 m (adaptive)
- **Costmap settings**: 
  - Local: 5x5 m, 0.06 m resolution
  - Global: Full map coverage, 0.05 m resolution
- **Obstacle detection**: Point cloud-based with voxel layer
- **Safety margins**: Inflation radius 4.0 m

### Vehicle Parameters

Edit `src/Robot_urdf/config/double_ackerman_params.yaml`:

```yaml
/**:
  ros__parameters:
    wheel_base: 2.39        # m
    track_width: 1.34       # m
    wheel_radius: 0.4       # m
    max_steering_angle: 0.7 # rad (~40°)
    max_wheel_speed: 10.0   # rad/s
```

### SLAM Configuration

Edit `src/autonomous/agriculture_robot_navigation/config/slam.yaml` for SLAM Toolbox parameters.

---

## 🤖 Robot Control Architecture

### Double Ackerman Steering Coordination

The system uses a custom rear steering commander to coordinate all four wheels:

1. **Front wheels**: Directly controlled by navigation controller
2. **Rear wheels**: Mirrored steering angles (opposite direction) calculated by `rear_steering_commander.py`
3. **Drive wheels**: All four wheels driven independently based on required velocities

### Control Flow

```
Nav2 Controller → cmd_vel → Rear Steering Commander → Individual Joint Commands
                                    ↓
                     [Front Left Steer, Front Right Steer]
                     [Rear Left Steer, Rear Right Steer]
                     [All Drive Wheels]
```

---

## 📊 Sensor Data Flow

```
Stereo Cameras (Left + Right)
    ↓
Point Cloud Generation
    ↓
┌──────────────┬─────────────────┐
│   RTAB-Map   │   Costmap 2D   │
│   (SLAM)     │   (Obstacles)   │
└──────────────┴─────────────────┘
         ↓              ↓
    Occupancy Grid  Local Costmap
         ↓              ↓
         └──────┬───────┘
                ↓
         Nav2 Planner
                ↓
         Nav2 Controller
                ↓
            cmd_vel
```

---

## 📈 Performance Metrics

### Typical Performance (Simulated)
- **Navigation frequency**: 30 Hz
- **Localization accuracy**: ±0.05 m
- **Max linear velocity**: 3.0 m/s
- **Max angular velocity**: 0.2 rad/s
- **Obstacle detection range**: 30 m (stereo cameras)

### Hardware Requirements (Recommended)
- **CPU**: Intel i5 (8th gen) or equivalent
- **RAM**: 8 GB minimum, 16 GB recommended
- **GPU**: NVIDIA GTX 1050 or better (for Gazebo simulation)
- **Storage**: 20 GB free space

---

## 🗺️ Roadmap

### Current Version: 1.0.5

### Planned Features
- [ ] Dynamic replanning with moving obstacles
- [ ] GPS integration for outdoor localization
- [ ] Field boundary detection and following
- [ ] Crop health monitoring integration
- [ ] Web-based monitoring interface

---

## 📚 References

### Related Papers and Projects
- **RTAB-Map**: [Real-Time Appearance-Based Mapping](http://introlab.github.io/rtabmap/)
- **Nav2**: [ROS 2 Navigation Framework](https://navigation.ros.org/)
- **Virtual Maize Field**: [Field Robot Event Simulation](https://github.com/FieldRobotEvent/virtual_maize_field)
- **Ackerman Steering**: [Four-Wheel Steering Systems](https://en.wikipedia.org/wiki/Steering#Four-wheel_steering)

---

## 👥 Contributors

- **Ebrahim Abdelghfar** - Project Lead and Main Developer
  - Email: ibrahim.abdelghafar@dakahlia.com

---

## 📄 License

This project is licensed under the **Apache License 2.0** - see individual package licenses for details:
- `agriculture_robot_navigation`: Apache 2.0
- `nav2_dynamic_window_pure_pursuit_controller`: Apache 2.0
- `vector_pursuit_controller`: Apache 2.0
- `virtual_maize_field`: GPL v3
- `rtabmap`: BSD
- `teb_local_planner`: BSD-3-Clause

---

## 🙏 Acknowledgments

This project integrates several open-source packages:
- **RTAB-Map** by IntRoLab
- **Nav2** by the ROS 2 Navigation Working Group
- **Virtual Maize Field** by the Field Robot Event team
- **TEB Local Planner** by Christoph Rösmann
- **Dynamic Window Pure Pursuit Controller** by Fumiya Ohnishi
- **Vector Pursuit Controller** by Black Coffee Robotics

Special thanks to the ROS 2 and agricultural robotics communities for their continuous support and contributions.

---

## 📞 Support

For issues, questions, or contributions:
- **GitHub Issues**: [Create an issue](https://github.com/ebrahimabdelghfar/double_ackerman_steering_stack/issues)
- **Discussions**: Use GitHub Discussions for general questions
- **Email**: ibrahim.abdelghafar@dakahlia.com

---

## 🔄 Version History

### Version 1.0.5 (Current - October 2025)
- Initial release with full double Ackerman steering support
- Nav2 integration with custom controllers
- RTAB-Map SLAM implementation
- Virtual maize field simulation
- Stereo camera-based obstacle detection
- Teleoperation support

---

**Note**: This project is under active development. Features and APIs may change. Please check the repository regularly for updates.

---

*Last Updated: October 6, 2025*

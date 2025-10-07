# Quick Start Guide - Double Steering Odometry

## Installation

```bash
cd ~/double_ackerman_steering_stack
colcon build --packages-select double_steering_odom
source install/setup.bash
```

## Basic Usage

### Launch Odometry Node
```bash
ros2 launch double_steering_odom double_steering_odom.launch.py
```

### Launch Complete System
```bash
# Terminal 1: Simulation
ros2 launch Robot_urdf gz_simulator_launch.py

# Terminal 2: Velocity Driver  
ros2 launch vel_driver vel_driver.launch.py

# Terminal 3: Odometry
ros2 launch double_steering_odom double_steering_odom.launch.py

# Terminal 4: Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Quick Checks

### Verify Joint States
```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

### Monitor Odometry
```bash
ros2 topic hz /odom
ros2 topic echo /odom
```

### Check TF
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

### Visualize in RViz
```bash
rviz2
# Add: Odometry (/odom), TF
# Set fixed frame: odom
```

## Configuration

Edit: `src/double_steering_odom/config/double_steering_odom.yaml`

**Key Parameters:**
- `wheel_base`: 2.39 m
- `track_width`: 1.34 m
- `wheel_radius`: 0.4 m
- `publish_tf`: true/false

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/joint_states` | sensor_msgs/JointState |
| Publish | `/odom` | nav_msgs/Odometry |

## TF Frames

```
odom → base_link
```

## Troubleshooting

### No odometry published?
1. Check `/joint_states` exists: `ros2 topic list | grep joint`
2. Verify node running: `ros2 node list | grep odom`
3. Check logs: `ros2 run double_steering_odom double_steering_odom_node --ros-args --log-level debug`

### Odometry drifting?
1. Verify robot parameters match actual robot
2. Check wheel radius accuracy
3. Ensure smooth joint state updates

### No TF published?
1. Set `publish_tf: true` in config
2. Check no other node publishes odom→base_link

## Debug Mode

```bash
ros2 run double_steering_odom double_steering_odom_node --ros-args --log-level debug
```

Shows detailed velocity and pose calculations.

## Documentation

Full docs: `src/double_steering_odom/README.md`

---

# double_steering_odom

ROS2 package for calculating odometry of double Ackerman steering robots from joint states.

## Overview

This package provides odometry calculation for robots with **double Ackerman steering** (4-wheel steering where both front and rear axles steer in opposite directions). It subscribes to `/joint_states` topic and publishes odometry information to `/odom` topic and optionally broadcasts the TF transform from `odom` to `base_link`.

## Features

- ✅ Accurate odometry calculation for double Ackerman steering kinematics
- ✅ Subscribes to standard `/joint_states` messages
- ✅ Publishes standard `/odom` messages (nav_msgs/Odometry)
- ✅ Optional TF broadcasting (odom → base_link)
- ✅ Configurable via YAML parameters
- ✅ Handles missing velocity data (calculates from position changes)
- ✅ Robust time step validation

## Installation

```bash
cd ~/double_ackerman_steering_stack
colcon build --packages-select double_steering_odom
source install/setup.bash
```

## Usage

### Launch the odometry node

```bash
ros2 launch double_steering_odom double_steering_odom.launch.py
```

### With custom configuration

```bash
ros2 launch double_steering_odom double_steering_odom.launch.py config_file:=/path/to/your/config.yaml
```

### Run standalone

```bash
ros2 run double_steering_odom double_steering_odom_node --ros-args --params-file /path/to/config.yaml
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions and velocities from the robot |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (pose and twist) |

### TF Transforms

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `odom` | `base_link` | Robot pose in odometry frame (optional, controlled by `publish_tf` parameter) |

## Parameters

All parameters are configured in `config/double_steering_odom.yaml`:

### Robot Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_base` | double | 2.39 | Distance between front and rear axles [m] |
| `track_width` | double | 1.34 | Distance between left and right wheels [m] |
| `wheel_radius` | double | 0.4 | Wheel radius [m] |

### Topic Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint_state_topic` | string | /joint_states | Input topic for joint states |
| `odom_topic` | string | /odom | Output topic for odometry |

### Frame Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_frame` | string | odom | Frame ID for odometry |
| `base_frame` | string | base_link | Frame ID for robot base |
| `publish_tf` | bool | true | Whether to publish TF transform |

### Joint Name Parameters

**Steering Joints** (measure steering angles):
- `fl_steer_joint`: Front left steering joint (default: `steer_front_left_link_j`)
- `fr_steer_joint`: Front right steering joint (default: `steer_front_right_link_j`)
- `rl_steer_joint`: Rear left steering joint (default: `steer_rear_left_link_j`)
- `rr_steer_joint`: Rear right steering joint (default: `steer_rear_right_link_j`)

**Wheel Joints** (measure wheel rotations):
- `fl_wheel_joint`: Front left wheel joint (default: `drive_front_left_link_j`)
- `fr_wheel_joint`: Front right wheel joint (default: `drive_front_right_link_j`)
- `rl_wheel_joint`: Rear left wheel joint (default: `drive_rear_left_link_j`)
- `rr_wheel_joint`: Rear right wheel joint (default: `drive_rear_right_link_j`)

## Double Ackerman Odometry Calculation

### Mathematical Model

The odometry is calculated using the kinematic model of double Ackerman steering:

1. **Read Joint States:**
   - Steering angles: $ δ_{fl}, δ_{fr}, δ_{rl}, δ_{rr}$ (from steering joints)
   - Wheel angular velocities: $ ω_{fl}, ω_{fr}, ω_{rl}, ω_{rr} $ (from wheel joints)

2. **Calculate Linear Velocities:**
  
  $$
  \begin{gather}
   v_{fl} = ω_{fl} × r \\ 
   v_{fr} = ω_{fr} × r \\
   v_{rl} = ω_{rl} × r \\
   v_{rr} = ω_{rr} × r \\
  \end{gather}
  $$

  where r is the wheel radius

3. **Calculate Average Values:**

  $$
  \begin{gather}
   δ_{front_{avg}} = (δ_{fl} + δ_{fr}) / 2 \\
   δ_{rear_{avg}}= (δ_{rl} + δ_{rr}) / 2 \\
   v_{front_{avg}} = (v_{fl} + v_{fr}) / 2 \\
   v_{rear_{avg}}= (v_{rl} + v_{rr}) / 2
  \end{gather}
  $$

4. **Calculate Robot Velocities:**

  For straight motion (δ ≈ 0):

  $$
  \begin{gather}
   ω = 0 \\
   v_x = v_{longitudinal} \\
   v_y = 0 \\
  \end{gather}
  $$

  For turning:
  
  $$
  \begin{gather}
    ω = (v_{front} × sin(δ_{front}) - v_{rear} × sin(δ_{rear})) / L \\
    v_x = v_{longitudinal} × cos((δ_{front} + δ_{rear}) / 4) \\
    v_y = v_{longitudinal} × sin((δ_{front} - δ_{rear}) / 4) \\
  \end{gather}
  $$

  where L is the wheelbase

5. **Integrate to Get Pose:**
   
  $$
  \begin{gather}
   Δx = (v_x × cos(θ) - v_y × sin(θ)) × Δt \\
   Δy = (v_x × sin(θ) + v_y × cos(θ)) × Δt \\
   Δθ = ω × Δt \\
   
   x += Δx \\
   y += Δy \\
   θ += Δθ
  \end{gather}
  $$

### Key Assumptions

- No wheel slip (pure rolling contact)
- Flat terrain
- Rigid body motion
- Front and rear axles steer in opposite directions (double Ackerman)

## Integration with Your Robot

### 1. Ensure Joint States are Published

Your robot simulation (Gazebo) should publish joint states:

```bash
ros2 topic echo /joint_states
```

You should see messages containing all 8 joints (4 steering + 4 wheels).

### 2. Check Joint Names Match

Verify that the joint names in your URDF match those in the config file:

```bash
ros2 topic echo /joint_states --once | grep name
```

If they differ, update `config/double_steering_odom.yaml` accordingly.

### 3. Launch with Simulation

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch Robot_urdf gz_simulator_launch.py

# Terminal 2: Launch odometry node
ros2 launch double_steering_odom double_steering_odom.launch.py

# Terminal 3: Visualize in RViz
rviz2
```

In RViz, add:
- Odometry display (topic: `/odom`)
- TF display
- Set fixed frame to `odom`

## Monitoring and Debugging

### Check Odometry Output

```bash
ros2 topic echo /odom
```

### Monitor TF Tree

```bash
ros2 run tf2_tools view_frames
```

This creates a PDF showing the TF tree. You should see: `odom → base_link`

### Enable Debug Logging

```bash
ros2 run double_steering_odom double_steering_odom_node --ros-args --log-level debug
```

Debug messages will show:
- Robot velocities
- Pose updates
- Steering angles
- Time steps

### Verify Odometry Accuracy

Drive the robot in a square or circle and check if it returns to the starting position. Deviations indicate:
- Incorrect wheel radius
- Incorrect wheelbase/track width
- Wheel slip in simulation
- Timing issues

## Troubleshooting

### Issue: No odometry published

**Solution:**
1. Check if `/joint_states` topic is being published: `ros2 topic list | grep joint`
2. Verify joint names match: `ros2 topic echo /joint_states --once`
3. Check node is running: `ros2 node list | grep odom`

### Issue: Odometry drifts quickly

**Solution:**
1. Verify robot parameters match actual robot dimensions
2. Check wheel radius is accurate
3. Ensure joint state velocities are provided or positions update smoothly
4. Reduce simulation time step in Gazebo

### Issue: TF transform not published

**Solution:**
1. Check `publish_tf` parameter is set to `true`
2. Verify no other node is publishing odom→base_link transform
3. Use `ros2 run tf2_ros tf2_echo odom base_link` to check transform

### Issue: Robot appears to slide sideways

**Solution:**
- This is normal for double Ackerman, as v_y (lateral velocity) is typically small but non-zero
- If excessive, check steering angle calculations and wheel velocities

## Example Usage Scenarios

### Scenario 1: With vel_driver

```bash
# Terminal 1: Simulation
ros2 launch Robot_urdf gz_simulator_launch.py

# Terminal 2: Velocity driver
ros2 launch vel_driver vel_driver.launch.py

# Terminal 3: Odometry
ros2 launch double_steering_odom double_steering_odom.launch.py

# Terminal 4: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Scenario 2: Recording Odometry

```bash
# Record odometry for later analysis
ros2 bag record /odom /joint_states /tf

# Play back
ros2 bag play <bag_file>
```

## Advanced Configuration

### Custom Covariance Values

Edit `src/double_steering_odom.cpp` and modify the covariance matrices in `publishOdometry()`:

```cpp
// Position covariance (tune based on your robot)
odom_msg.pose.covariance[0] = 0.001;  // x variance
odom_msg.pose.covariance[7] = 0.001;  // y variance
odom_msg.pose.covariance[35] = 0.01;  // yaw variance
```

### Integration with Robot Localization

This odometry can be fused with other sensors using `robot_localization` package:

```bash
sudo apt install ros-humble-robot-localization
```

Configure `ekf_localization_node` to fuse wheel odometry with IMU and GPS.

## Package Structure

```
double_steering_odom/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── double_steering_odom.yaml
├── include/
│   └── double_steering_odom/
│       └── double_steering_odom.hpp
├── launch/
│   └── double_steering_odom.launch.py
└── src/
    ├── double_steering_odom.cpp
    └── double_steering_odom_node.cpp
```

## Contributing

Feel free to submit issues or pull requests to improve the odometry calculation or add features.

## License

MIT License

## Author

Ebrahim Abdelghaffar (ebrahimabdelghfar550@gmail.com)

## References

1. Double Ackerman Steering Kinematics
2. ROS2 nav_msgs/Odometry message specification
3. TF2 for coordinate frame transformations

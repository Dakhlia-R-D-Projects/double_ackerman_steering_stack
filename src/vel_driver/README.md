# vel_driver - ROS2 Package

This package has been converted from ROS1 to ROS2. It provides a velocity driver for **double Ackerman steering** vehicles in Gazebo simulation.

## Features

- Converts Twist commands to individual wheel steering and rotation commands
- Supports 4-wheel double Ackerman steering vehicles (both front and rear axles steer)
- Implements proper Ackerman steering geometry for realistic vehicle motion
- Configurable vehicle parameters via YAML file

## Building

```bash
colcon build --packages-select vel_driver
```

## Running

```bash
ros2 launch vel_driver vel_driver.launch.py
```

## Configuration

Edit `config/vel_driver.yaml` to modify:
- Topic names for input/output
- Vehicle parameters (wheelbase, track width, wheel radius)
- Maximum steering angle limits
- Velocity threshold for zero-velocity detection

## Changes from ROS1

- Updated to use `rclcpp` instead of `roscpp`
- Changed message types to ROS2 format (`geometry_msgs::msg::Twist`)
- Updated launch file to Python format
- Modified parameter handling to use ROS2 parameter system
- Updated logging to use `RCLCPP_*` macros
- **Converted from 4WIDS (swerve drive) to double Ackerman steering kinematics**

## Overview

This package converts a 2DoF twist message (vx, yaw_rate) to 8DoF vehicle command message and publishes it.  
Gazebo will subscribe the 8DoF vehicle command message and apply it to the double Ackerman steering vehicle model in the simulation.

**Double Ackerman Steering**: Both front and rear axles can steer, with the rear axle steering in the opposite direction to the front axle. This allows for:
- Tighter turning radius
- Better maneuverability in confined spaces
- Reduced tire scrub during turns
- More stable turning characteristics


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /cmd_vel        | geometry_msgs/Twist  | Target linear velocity (vx) and angular velocity (yaw_rate) to follow. Note: vy (lateral velocity) is typically 0 for Ackerman steering. |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /fwids/front_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front left wheel (calculated using Ackerman geometry). |
| /fwids/front_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front left wheel. |
| /fwids/front_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front right wheel (calculated using Ackerman geometry). |
| /fwids/front_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front right wheel. |
| /fwids/rear_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear left wheel (opposite sign to front for double Ackerman). |
| /fwids/rear_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear left wheel. |
| /fwids/rear_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear right wheel (opposite sign to front for double Ackerman). |
| /fwids/rear_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear right wheel. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| wheel_base                  | double | Distance between front and rear axles [m] (default: 2.39) |
| track_width                 | double | Distance between left and right wheels [m] (default: 1.34) |
| wheel_radius                | double | Wheel radius [m] (default: 0.4) |
| max_steering_angle          | double | Maximum steering angle limit [rad] (default: 0.7 ≈ 40°) |
| velocity_threshold          | double | Threshold for zero velocity detection [m/s or rad/s] (default: 1e-2) |
| control_cmd_vel_topic       | string | Input topic for velocity commands (default: /cmd_vel) |
| front_left_steer_cmd_topic  | string | Output topic for front left steering (default: /fwids/front_left_steer_rad/command) |
| front_right_steer_cmd_topic | string | Output topic for front right steering (default: /fwids/front_right_steer_rad/command) |
| rear_left_steer_cmd_topic   | string | Output topic for rear left steering (default: /fwids/rear_left_steer_rad/command) |
| rear_right_steer_cmd_topic  | string | Output topic for rear right steering (default: /fwids/rear_right_steer_rad/command) |
| front_left_rotor_cmd_topic  | string | Output topic for front left wheel velocity (default: /fwids/front_left_rotor_radpersec/command) |
| front_right_rotor_cmd_topic | string | Output topic for front right wheel velocity (default: /fwids/front_right_rotor_radpersec/command) |
| rear_left_rotor_cmd_topic   | string | Output topic for rear left wheel velocity (default: /fwids/rear_left_rotor_radpersec/command) |
| rear_right_rotor_cmd_topic  | string | Output topic for rear right wheel velocity (default: /fwids/rear_right_rotor_radpersec/command) |


## Usage
To launch the node individually, run the following commands.
```bash
# Build the package
colcon build --packages-select vel_driver

# Source the workspace
source install/setup.bash

# Launch the driver
ros2 launch vel_driver vel_driver.launch.py
```

You can also specify a custom parameter file:
```bash
ros2 launch vel_driver vel_driver.launch.py vel_driver_param_path:=/path/to/your/config.yaml
```

## Double Ackerman Steering Kinematics

The conversion between 2DoF twist message (vx, yaw_rate) and 8DoF vehicle command message is based on double Ackerman steering geometry:

### Front Axle Steering:
- **Inner wheel angle**: `θ_inner = atan(wheelbase / (turning_radius - track_width/2))`
- **Outer wheel angle**: `θ_outer = atan(wheelbase / (turning_radius + track_width/2))`

### Rear Axle Steering:
- Rear wheels steer in the **opposite direction** to the front wheels
- This creates a "crab-like" motion that reduces the turning radius

### Wheel Velocities:
- Each wheel's angular velocity is calculated based on its distance from the instantaneous center of rotation
- Accounts for different path lengths of inner and outer wheels during turns

### Key Assumptions:
1. No tire slip (pure rolling contact)
2. Rigid body motion
3. Flat terrain
4. Ackerman steering constraints are maintained

## Benefits of Double Ackerman

1. **Reduced Turning Radius**: Can turn in tighter spaces than single Ackerman
2. **Lower Tire Wear**: Proper Ackerman geometry reduces tire scrubbing
3. **Better Maneuverability**: Ideal for agricultural and industrial applications
4. **Stable Turning**: Both axles contribute to directional control

# Changes Made to vel_driver Package for Double Ackerman Steering

## Summary
The vel_driver package has been converted from a 4-wheel independent steering (4WIDS/swerve drive) system to a **double Ackerman steering** system suitable for agricultural robots and vehicles with both front and rear axle steering.

## Key Changes

### 1. Header File (`include/vel_driver/vel_driver.hpp`)
**Removed:**
- `extension_length` parameter (not needed for Ackerman)
- `l_f, l_r, d_l, d_r` parameters (swerve-specific center of mass offsets)
- `extensionLengthCallback()` function

**Added:**
- `wheel_base` - Distance between front and rear axles
- `track_width` - Distance between left and right wheels
- `wheel_radius` - Wheel radius
- `max_steering_angle` - Maximum steering angle limit
- `calculateAckermannAngles()` - Helper function for Ackerman geometry calculations

### 2. Implementation File (`src/vel_driver.cpp`)

#### Parameter Updates:
**Old Parameters (Swerve Drive):**
```yaml
l_f: 0.5              # front axle to CoM
l_r: 0.5              # rear axle to CoM
d_l: 0.5              # left wheel to CoM
d_r: 0.5              # right wheel to CoM
tire_radius: 0.2      # wheel radius
```

**New Parameters (Double Ackerman):**
```yaml
wheel_base: 2.39      # distance between axles
track_width: 1.34     # distance between wheels
wheel_radius: 0.4     # wheel radius
max_steering_angle: 0.7  # max steering limit (~40°)
velocity_threshold: 1e-2 # zero velocity threshold
```

#### Kinematic Model Changes:
**Old Approach (Swerve Drive):**
- Each wheel calculated independently
- Steering angles from arctangent of velocity components
- Complex wrapping logic for each wheel
- Supported omnidirectional motion (vx, vy, omega)

**New Approach (Double Ackerman):**
- Turning radius calculated from linear and angular velocity
- Front axle: Inner/outer wheel angles using Ackerman geometry
- Rear axle: Opposite steering angles for double Ackerman
- Wheel speeds calculated based on distance from turning center
- Primarily supports forward/backward motion with rotation

### 3. Configuration File (`config/vel_driver.yaml`)

**Before:**
```yaml
l_f: 0.5
l_r: 0.5
d_l: 0.5
d_r: 0.5
tire_radius: 0.2
```

**After:**
```yaml
wheel_base: 2.39
track_width: 1.34
wheel_radius: 0.4
max_steering_angle: 0.7
velocity_threshold: 1e-2
```

### 4. Documentation (`README.md`)

**Updated Sections:**
- Package description (swerve → double Ackerman)
- Features list (added Ackerman geometry)
- Overview section (explained double Ackerman benefits)
- Parameters table (completely restructured)
- Added kinematics section with formulas
- Added benefits of double Ackerman section
- Updated usage instructions for ROS2

## Double Ackerman Steering Explained

### What is Double Ackerman?
Unlike standard Ackerman steering (only front wheels steer), double Ackerman steering allows **both front and rear axles** to steer, with the rear axle steering in the **opposite direction** to the front.

### Advantages:
1. **Tighter Turning Radius**: Can maneuver in confined spaces
2. **Reduced Tire Scrub**: Proper Ackerman geometry reduces tire wear
3. **Better Stability**: Both axles contribute to directional control
4. **Agricultural Applications**: Ideal for field work and row navigation

### Kinematics:
For a given turning radius `R`:
- **Front Inner Wheel**: `θ = atan(L / (R - W/2))`
- **Front Outer Wheel**: `θ = atan(L / (R + W/2))`
- **Rear Wheels**: Negative of front wheel angles

Where:
- `L` = wheelbase (distance between axles)
- `W` = track width (distance between wheels)
- `R` = turning radius

## Testing Recommendations

1. **Straight Line Motion**: Test with `omega = 0`
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

2. **Turning Motion**: Test with various angular velocities
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
   ```

3. **Verify Steering Angles**: Monitor steering commands
   ```bash
   ros2 topic echo /fwids/front_left_steer_rad/command
   ros2 topic echo /fwids/rear_left_steer_rad/command
   ```

4. **Check Wheel Velocities**: Ensure differential speeds during turns
   ```bash
   ros2 topic echo /fwids/front_left_rotor_radpersec/command
   ros2 topic echo /fwids/front_right_rotor_radpersec/command
   ```

## Integration with Your Robot

The parameters in `vel_driver.yaml` match your robot's specifications from `Robot_urdf/config/double_ackerman_params.yaml`:
- Wheelbase: 2.39 m
- Track width: 1.34 m
- Wheel radius: 0.4 m
- Max steering angle: 0.7 rad (~40°)

The topic names are configured to work with your Gazebo bridge setup defined in `Robot_urdf/config/gz_bridge.yaml`.

## Build and Test

```bash
# Navigate to workspace
cd /home/ebrahim/double_ackerman_steering_stack

# Build the package
colcon build --packages-select vel_driver

# Source the workspace
source install/setup.bash

# Launch the driver
ros2 launch vel_driver vel_driver.launch.py

# In another terminal, test with teleop or manual commands
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

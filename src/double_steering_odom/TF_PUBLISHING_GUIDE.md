# TF Publishing Configuration Guide

## Overview

The `double_steering_odom` package can optionally publish the TF transform from `odom` to `base_link`. This is controlled by the `publish_tf` parameter in the configuration file.

## Configuration Parameter

**File:** `config/double_steering_odom.yaml`

```yaml
publish_tf: true  # or false
```

## When to Enable TF Publishing (`publish_tf: true`)

✅ **Use this setting when:**

1. **Standalone Odometry:** You're using wheel odometry as your primary localization source
2. **Simple Setup:** You don't need sensor fusion with IMU, GPS, or other sensors
3. **Simulation Only:** Testing in Gazebo without complex localization
4. **Direct Control:** You want this node to directly control the odom→base_link transform

**Example scenarios:**
- Testing robot motion in simulation
- Basic navigation without sensor fusion
- Development and debugging
- Simple autonomous navigation

## When to Disable TF Publishing (`publish_tf: false`)

❌ **Use this setting when:**

1. **Using `robot_localization`:** When fusing wheel odometry with IMU, GPS, etc.
2. **Custom Localization:** You have your own localization node publishing this transform
3. **Multiple Odometry Sources:** Avoiding TF conflicts when multiple nodes might publish
4. **SLAM/Mapping:** Using AMCL, Cartographer, or RTAB-Map which publish their own transforms

**Example scenarios:**
- Production robot with sensor fusion
- Using EKF or UKF for state estimation
- Running SLAM algorithms
- Multiple odometry sources (visual, laser, wheel)

## Usage Examples

### Example 1: Standalone Odometry (TF Enabled)

**Config:** `config/double_steering_odom.yaml`
```yaml
publish_tf: true
```

**Launch:**
```bash
ros2 launch double_steering_odom double_steering_odom.launch.py
```

**TF Tree:**
```
map (if using AMCL/SLAM)
 └─ odom (published by this node)
     └─ base_link
         └─ [sensors, wheels, etc.]
```

**Verification:**
```bash
# Check TF is being published
ros2 run tf2_ros tf2_echo odom base_link

# Should show continuously updating transform
```

### Example 2: With robot_localization (TF Disabled)

**Config:** `config/double_steering_odom.yaml`
```yaml
publish_tf: false  # Let robot_localization publish this
```

**Launch:**
```bash
# Terminal 1: Odometry (publishes /odom message only, no TF)
ros2 launch double_steering_odom double_steering_odom.launch.py

# Terminal 2: robot_localization (publishes TF based on fused data)
ros2 launch robot_localization ekf.launch.py
```

**TF Tree:**
```
map
 └─ odom (published by robot_localization's EKF)
     └─ base_link
         └─ [sensors]
```

**robot_localization config snippet:**
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    
    # Input sources
    odom0: /odom  # From double_steering_odom (no TF)
    odom0_config: [false, false, false,  # x, y, z
                   false, false, false,  # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    
    imu0: /imu/data
    imu0_config: [false, false, false,   # x, y, z
                  false, false, true,    # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  false, false, true,    # vroll, vpitch, vyaw
                  true,  true,  false]   # ax, ay, az
```

## Checking Current Configuration

### At Runtime

When the node starts, it will log:

**If TF enabled:**
```
[double_steering_odom]: TF broadcasting enabled: odom -> base_link
[double_steering_odom]: Double Steering Odometry node started
```

**If TF disabled:**
```
[double_steering_odom]: TF broadcasting disabled
[double_steering_odom]: Double Steering Odometry node started
```

### Verify TF Publishing

**Check if TF is being published:**
```bash
# List all TF frames
ros2 run tf2_tools view_frames

# Echo specific transform (will error if not published)
ros2 run tf2_ros tf2_echo odom base_link

# Monitor TF tree in real-time
ros2 run rqt_tf_tree rqt_tf_tree
```

### Check What's Publishing TF

```bash
# See all TF publishers
ros2 topic info /tf

# Monitor TF messages
ros2 topic echo /tf --once
```

## Common Scenarios

### Scenario 1: Running Multiple Robots

If running multiple robots in simulation, ensure each has unique frame names:

**Robot 1:**
```yaml
odom_frame: robot1/odom
base_frame: robot1/base_link
publish_tf: true
```

**Robot 2:**
```yaml
odom_frame: robot2/odom
base_frame: robot2/base_link
publish_tf: true
```

### Scenario 2: Switching from Simulation to Real Robot

**Simulation (Gazebo):**
```yaml
publish_tf: true  # Simple setup for testing
```

**Real Robot (with IMU and sensor fusion):**
```yaml
publish_tf: false  # Let robot_localization handle TF
```

### Scenario 3: Using with Nav2

**For Nav2, you typically want:**
```yaml
publish_tf: true  # If using wheel odometry only
# OR
publish_tf: false  # If using robot_localization
```

Nav2 requires these frames:
```
map → odom → base_link → [sensors]
```

- `map→odom`: Published by AMCL or SLAM
- `odom→base_link`: Published by this node OR robot_localization
- `base_link→sensors`: Published by robot_state_publisher

## Troubleshooting

### Problem: "Lookup would require extrapolation into the future"

**Cause:** TF transform not being published

**Solutions:**
1. Check `publish_tf: true` in config
2. Verify node is running: `ros2 node list | grep odom`
3. Check joint_states are being received
4. Verify no TF conflicts (two nodes publishing same transform)

### Problem: TF transform is delayed or jumpy

**Possible causes:**
1. Low joint_state publication rate
2. High computational load
3. Network delays (if using remote connection)

**Solutions:**
1. Check joint_state rate: `ros2 topic hz /joint_states`
2. Reduce system load
3. Increase odometry update rate if possible

### Problem: Two nodes publishing same transform

**Error message:**
```
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp
```

**Solution:**
Set `publish_tf: false` in one of the nodes

**Check who's publishing:**
```bash
ros2 topic info /tf
ros2 topic echo /tf --once | grep frame_id
```

## Best Practices

1. **Document your choice:** Add comments in launch files explaining why TF is enabled/disabled

2. **Consistent configuration:** Use the same setting across development and production if possible

3. **Version control:** Track config changes when switching between TF modes

4. **Testing:** Always verify TF tree after changing this setting:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

5. **Integration:** When integrating with other packages, check their TF requirements first

## Parameter Override

You can override the config file setting at launch time:

```bash
ros2 launch double_steering_odom double_steering_odom.launch.py \
  config_file:=/path/to/custom/config.yaml
```

Or use parameter override:

```bash
ros2 run double_steering_odom double_steering_odom_node \
  --ros-args -p publish_tf:=false
```

## Summary

| Scenario | publish_tf | Who Publishes odom→base_link |
|----------|------------|------------------------------|
| Simple odometry | `true` | double_steering_odom |
| With robot_localization | `false` | robot_localization |
| With custom fusion | `false` | Your custom node |
| Testing/Development | `true` | double_steering_odom |
| Production with sensors | `false` | Sensor fusion node |

---

**Default Setting:** `publish_tf: true` (enabled)

**Recommendation:** Keep enabled for simulation and basic testing. Disable when using sensor fusion or custom localization.

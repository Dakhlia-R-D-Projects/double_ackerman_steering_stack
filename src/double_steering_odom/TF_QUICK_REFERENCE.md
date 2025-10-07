# TF Publishing - Quick Reference

## Configuration

**File:** `config/double_steering_odom.yaml`

```yaml
publish_tf: true   # Enable TF publishing
# or
publish_tf: false  # Disable TF publishing
```

## Quick Decision Guide

```
Do you have IMU, GPS, or other localization sensors?
│
├─ YES → Are you using robot_localization or sensor fusion?
│   │
│   ├─ YES → publish_tf: false
│   │         (Let sensor fusion handle TF)
│   │
│   └─ NO → publish_tf: true
│            (Use wheel odometry only)
│
└─ NO → publish_tf: true
         (Use wheel odometry only)
```

## Common Setups

### ✅ Enable TF (`publish_tf: true`)
- Simulation testing
- Wheel odometry only
- No sensor fusion
- Development phase

### ❌ Disable TF (`publish_tf: false`)
- Using robot_localization
- Using EKF/UKF fusion
- Custom localization node
- Running SLAM

## Verify Settings

```bash
# Check if TF is being published
ros2 run tf2_ros tf2_echo odom base_link

# View TF tree
ros2 run tf2_tools view_frames

# Check node logs
ros2 run double_steering_odom double_steering_odom_node
# Look for: "TF broadcasting enabled" or "TF broadcasting disabled"
```

## Runtime Override

```bash
# Override to disable
ros2 run double_steering_odom double_steering_odom_node \
  --ros-args -p publish_tf:=false

# Override to enable  
ros2 run double_steering_odom double_steering_odom_node \
  --ros-args -p publish_tf:=true
```

## Example Configurations

### Standalone
```yaml
publish_tf: true
odom_frame: odom
base_frame: base_link
```

### With robot_localization
```yaml
publish_tf: false  # robot_localization will publish TF
odom_frame: odom
base_frame: base_link
```

### Multiple Robots
```yaml
# Robot 1
publish_tf: true
odom_frame: robot1/odom
base_frame: robot1/base_link

# Robot 2
publish_tf: true
odom_frame: robot2/odom
base_frame: robot2/base_link
```

---
**Default:** `publish_tf: true`

**Full Guide:** See `TF_PUBLISHING_GUIDE.md`

# Kinematic Feasibility Improvements for Double Ackerman Steering

## Overview
This document explains the kinematic constraints and improvements made to ensure the vel_driver properly handles physically feasible motion for a double Ackerman steering vehicle.

## Key Improvements

### 1. Minimum Turning Radius Constraint

**Problem:** The original implementation directly used `turning_radius = v_x / omega`, which could result in kinematically infeasible commands when the requested angular velocity is too high relative to linear velocity.

**Solution:** Calculate and enforce the minimum kinematically feasible turning radius:

```cpp
// Minimum turning radius based on maximum steering angle
float min_turning_radius = wheel_base / tan(max_steering_angle);
float safe_min_radius = min_turning_radius + track_width / 2.0;

if (abs(turning_radius) < safe_min_radius) {
    // Limit to minimum feasible radius
    turning_radius = (turning_radius > 0) ? safe_min_radius : -safe_min_radius;
}
```

**Why this works:**
- For Ackerman steering: `tan(δ) = L / R` where δ is steering angle, L is wheelbase, R is turning radius
- Maximum steering angle δ_max defines minimum turning radius: `R_min = L / tan(δ_max)`
- We add half the track width as safety margin to account for wheel positions

### 2. Safe Division Prevention

**Problem:** When turning radius approaches or becomes smaller than half the track width, the Ackerman angle calculation results in division by very small or negative numbers.

**Solution:** Added safety check in `calculateAckermannAngles()`:

```cpp
if (abs_radius <= half_track) {
    RCLCPP_WARN(...);
    // Set to maximum steering angle with appropriate ratio
    left_angle = (turning_radius > 0) ? max_steering_angle : -max_steering_angle;
    right_angle = left_angle * 0.7; // Outer wheel smaller angle
    return;
}
```

**Physical meaning:** 
- The instantaneous center of rotation (ICR) cannot be inside the vehicle footprint
- If turning radius < track_width/2, the ICR would be between the left and right wheels, which is physically impossible for Ackerman steering

### 3. Accurate Wheel Velocity Calculation

**Problem:** Previous wheel velocity calculation had simplified geometry that didn't properly account for wheel positions relative to the turning center.

**Solution:** Calculate exact distance from ICR to each wheel:

```cpp
// For each wheel, calculate distance from ICR using Pythagorean theorem
r_fl = sqrt((turning_radius - half_track)² + (half_wheelbase)²)
r_fr = sqrt((turning_radius + half_track)² + (half_wheelbase)²)
// ... and so on

// Linear velocity at each wheel
v_wheel = r_wheel * omega

// Convert to wheel angular velocity
wheel_speed = v_wheel / wheel_radius
```

**Why this is correct:**
- Each wheel travels in a circular arc around the ICR
- The radius of this arc depends on the wheel's position
- Inner wheels (closer to ICR) travel slower than outer wheels
- Front and rear wheels at the same lateral position have the same distance to ICR

## Physical Constraints of Double Ackerman Steering

### Geometric Relationships

1. **Instantaneous Center of Rotation (ICR):**
   - Located on the line extending from the vehicle's longitudinal centerline
   - Distance from vehicle center = turning radius
   - All wheel axes must pass through this point (in pure rolling)

2. **Steering Angle Limits:**
   - Maximum steering angle: 0.7 rad (≈40°)
   - Minimum turning radius: `L / tan(0.7) ≈ 2.39 / 0.842 ≈ 2.84 m`
   - With safety margin: `≈ 3.51 m` (adding track_width/2)

3. **Double Ackerman Behavior:**
   - Front wheels steer in one direction
   - Rear wheels steer in opposite direction
   - This reduces the effective turning radius by approximately 2x compared to single Ackerman
   - The vehicle "crabs" around the turn

### Kinematic Constraints

1. **No-slip condition:** Each wheel must roll without lateral slipping
2. **Ackerman condition:** All wheel rotation axes must intersect at the ICR
3. **Velocity constraint:** `v_wheel = r_wheel × ω_vehicle`
4. **Steering angle constraint:** `-δ_max ≤ δ ≤ δ_max` for all wheels

## Validation Tests

### Test 1: Maximum Steering Test
```bash
# Command tight turn at slow speed
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Expected: Turning radius ≈ 1.0 m (commanded)
# Result: Limited to ≈ 3.51 m (minimum feasible)
# Steering angles should be at or near max_steering_angle
```

### Test 2: Moderate Turn
```bash
# Command moderate turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Expected: Turning radius ≈ 6.67 m
# Result: Should be achievable without limiting
# Inner wheels should have larger angles than outer wheels
```

### Test 3: Straight Line
```bash
# Command straight motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Expected: All steering angles = 0
# All wheel speeds equal
```

## Monitoring and Debugging

### Key Topics to Monitor

1. **Steering Angles:**
```bash
ros2 topic echo /fwids/front_left_steer_rad/command
ros2 topic echo /fwids/front_right_steer_rad/command
ros2 topic echo /fwids/rear_left_steer_rad/command
ros2 topic echo /fwids/rear_right_steer_rad/command
```

Expected relationships:
- Front and rear should have opposite signs
- Inner wheels should have larger magnitude than outer wheels
- All should be within [-0.7, 0.7] rad

2. **Wheel Velocities:**
```bash
ros2 topic echo /fwids/front_left_rotor_radpersec/command
ros2 topic echo /fwids/front_right_rotor_radpersec/command
```

Expected relationships:
- During left turn: right wheels (outer) faster than left wheels (inner)
- During right turn: left wheels (outer) faster than right wheels (inner)
- All same speed during straight line

### Debug Logging

Enable debug logging to see detailed calculations:
```bash
ros2 run vel_driver vel_driver_node --ros-args --log-level debug
```

Look for messages like:
- "Requested turning radius too tight, limited to X m"
- "Turning radius too small compared to track width"
- "Turning radius: X, Steer: FL=X FR=X RL=X RR=X"

## Vehicle Parameters

Current configuration (from `double_ackerman_params.yaml`):
```yaml
wheel_base: 2.39 m       # Distance between axles
track_width: 1.34 m      # Distance between wheels
wheel_radius: 0.4 m      # Wheel radius
max_steering_angle: 0.7 rad  # ≈40 degrees
```

Calculated limits:
- **Minimum turning radius:** ~2.84 m (theoretical)
- **Safe minimum turning radius:** ~3.51 m (with margin)
- **Maximum lateral acceleration:** Depends on speed and radius

## References

1. Ackerman Steering Geometry: Classical vehicle dynamics
2. Double Ackerman: Enhanced maneuverability for agricultural vehicles
3. Kinematic constraints: Assuming no tire slip (pure rolling)

## Troubleshooting

### Issue: Robot doesn't turn as tight as expected
- Check if turning radius is being limited (look for debug messages)
- Verify max_steering_angle is set correctly
- Consider if physical robot matches URDF parameters

### Issue: Wheels slip during turns
- May indicate turning radius is still too tight
- Increase safe_min_radius margin
- Check ground friction in simulation

### Issue: Inconsistent steering angles
- Verify all steering angle topics are being published
- Check for NaN values in input commands
- Ensure wheel_base and track_width match actual robot

## Summary

The improved implementation ensures:
1. ✅ Kinematically feasible turning radius
2. ✅ Safe division prevention
3. ✅ Accurate wheel velocity distribution
4. ✅ Proper Ackerman geometry relationships
5. ✅ Realistic vehicle motion

The vehicle will now properly limit infeasible commands while maintaining smooth, realistic motion within its physical constraints.

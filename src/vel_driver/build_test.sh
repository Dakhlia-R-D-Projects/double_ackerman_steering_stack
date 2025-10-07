#!/bin/bash
# Test build script for vel_driver package

echo "=========================================="
echo "Building vel_driver package..."
echo "=========================================="

cd /home/ebrahim/double_ackerman_steering_stack

# Build only vel_driver package
colcon build --packages-select vel_driver --symlink-install

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "✓ Build successful!"
    echo "=========================================="
    echo ""
    echo "To test the driver, run:"
    echo "  source install/setup.bash"
    echo "  ros2 launch vel_driver vel_driver.launch.py"
    echo ""
    echo "In another terminal, you can send test commands:"
    echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\"
    echo "    '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'"
else
    echo "=========================================="
    echo "✗ Build failed!"
    echo "=========================================="
    exit 1
fi

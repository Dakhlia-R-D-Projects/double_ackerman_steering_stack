#include "double_steering_odom/double_steering_odom.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<double_steering_odom::DoubleSteeringOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

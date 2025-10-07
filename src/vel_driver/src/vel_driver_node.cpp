#include "vel_driver/vel_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo::VelDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};

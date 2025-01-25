#include <rclcpp/rclcpp.hpp>
#include <fiesta_pkg/Fiesta.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<fiesta::Fiesta<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

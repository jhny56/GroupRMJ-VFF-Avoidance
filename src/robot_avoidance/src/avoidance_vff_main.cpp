//#include "./../include/vff_avoidance/avoidance.hpp"
#include "./vff_avoidance/avoidance.cpp"
#include <cmath>
#include <algorithm>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
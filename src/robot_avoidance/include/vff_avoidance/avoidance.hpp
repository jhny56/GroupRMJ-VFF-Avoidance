#ifndef AVOIDANCE__AVOIDANCE_NODE_HPP_
#define AVOIDANCE__AVOIDANCE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

class AvoidanceNode : public rclcpp::Node
{
public:
  AvoidanceNode();

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  std::vector<float> calculate_repulsion(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  std::vector<float> calculate_attraction();
  std::vector<float> calculate_resultant(const std::vector<float>& attraction, const std::vector<float>& repulsion);
  void publish_markers(const std::vector<float>& attraction, const std::vector<float>& repulsion, const std::vector<float>& resultant);
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // double current_angle_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  void load_parameters();

  float repulsion_gain_;
  float min_threshold_range_;
  float max_linear_velocity_;
  float max_angular_velocity_;
};

#endif  // AVOIDANCE__AVOIDANCE_NODE_HPP_

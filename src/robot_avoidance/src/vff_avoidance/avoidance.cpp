#include "avoidance.hpp"
#include <cmath>
#include <algorithm>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

AvoidanceNode::AvoidanceNode() : Node("avoidance_node")
{
  load_parameters();

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&AvoidanceNode::lidar_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&AvoidanceNode::odom_callback, this, std::placeholders::_1));

   timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&AvoidanceNode::timer_callback, this));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vector_markers", 10);

  latest_scan_ = nullptr; 
}

void AvoidanceNode::load_parameters()
{
  declare_parameter("repulsion_gain", 0.2f);
  declare_parameter("min_threshold_range", 0.4f);
  declare_parameter("max_linear_velocity", 0.1f);
  declare_parameter("max_angular_velocity", 0.8f);

  repulsion_gain_ = get_parameter("repulsion_gain").as_double();
  min_threshold_range_ = get_parameter("min_threshold_range").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();


  RCLCPP_INFO(this->get_logger(), "Parameters loaded: repulsion_gain=%.2f, min_threshold_range=%.2f, max_linear_velocity=%.2f, max_angular_velocity=%.2f",
              repulsion_gain_, min_threshold_range_, max_linear_velocity_, max_angular_velocity_);
}

void AvoidanceNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;

  auto repulsion = calculate_repulsion(msg);
  auto attraction = calculate_attraction();
  auto resultant = calculate_resultant(attraction, repulsion);

  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = resultant[2];
  cmd_vel_msg.linear.x = resultant[3];

  cmd_vel_pub_->publish(cmd_vel_msg);
  publish_markers(attraction, repulsion, resultant);
}

void AvoidanceNode::timer_callback()
{
  if (latest_scan_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "No laser scan data received yet.");
    return;
  }

  auto repulsion = calculate_repulsion(latest_scan_);
  auto attraction = calculate_attraction();
  auto resultant = calculate_resultant(attraction, repulsion);

  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = resultant[2];
  cmd_vel_msg.linear.x = resultant[3];

  cmd_vel_pub_->publish(cmd_vel_msg);
  publish_markers(attraction, repulsion, resultant);
}

void AvoidanceNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // double roll, pitch, yaw;

  // // Extract the orientation quaternion
  // auto orientation_q = msg->pose.pose.orientation;

  // // Convert quaternion to Euler angles
  // tf2::Quaternion q(
  //   orientation_q.x,
  //   orientation_q.y,
  //   orientation_q.z,
  //   orientation_q.w
  // );
  // tf2::Matrix3x3 m(q);
  // m.getRPY(roll, pitch, yaw);

  // current_angle_ = yaw;
}

std::vector<float> AvoidanceNode::calculate_repulsion(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float repulsion_x = 0.0;
    float repulsion_y = 0.0;
    float repulsion_gain = 0.1;

    auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    float min_range = *min_it;
    size_t min_index = std::distance(msg->ranges.begin(), min_it);

    if (min_range < min_threshold_range_) 
    {
        float angle = msg->angle_min + min_index * msg->angle_increment;
        repulsion_x = repulsion_gain*(1.0 / min_range) * std::cos(angle);
        repulsion_y = repulsion_gain*(1.0 / min_range) * std::sin(angle);
    }

    return {-repulsion_x, -repulsion_y};
}

std::vector<float> AvoidanceNode::calculate_attraction()
{
  float attraction_x = 0.2; // Constant forward attraction
  float attraction_y = 0.0;

  return {attraction_x, attraction_y};
}

std::vector<float> AvoidanceNode::calculate_resultant(const std::vector<float>& attraction, const std::vector<float>& repulsion)
{
  float resultant_x = attraction[0] + repulsion[0];
  float resultant_y = attraction[1] + repulsion[1];

  float desired_angle = std::atan2(resultant_y, resultant_x);

  float Kp = 0.3;
  float angular_velocity = Kp * desired_angle;
  float linear_velocity = std::sqrt(resultant_x * resultant_x + resultant_y * resultant_y);
  RCLCPP_INFO(this->get_logger(), "Calculated Angular Velocity: %f", angular_velocity);
  RCLCPP_INFO(this->get_logger(), "Desired Angle: %f", desired_angle);


  angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
  linear_velocity = std::clamp(linear_velocity, -max_linear_velocity_, max_linear_velocity_);


  return {resultant_x, resultant_y, angular_velocity, linear_velocity};
}

void AvoidanceNode::publish_markers(const std::vector<float>& attraction, const std::vector<float>& repulsion, const std::vector<float>& resultant)
{
  auto create_marker = [this](const std::vector<float>& vector, const std::string& ns, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    geometry_msgs::msg::Point start, end;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    end.x = vector[0];
    end.y = vector[1];
    end.z = 0;
    marker.points.push_back(start);
    marker.points.push_back(end);
    return marker;
  };

  // publish markers
  marker_pub_->publish(create_marker(attraction, "attraction", 0, 0.0, 0.0, 1.0)); //green
  marker_pub_->publish(create_marker(repulsion, "repulsion", 1, 1.0, 0.0, 0.0)); // red
  marker_pub_->publish(create_marker(resultant, "resultant", 2, 0.0, 1.0, 0.0)); // blue
}



#include "avoidance/avoidance_node.hpp"
#include <cmath>
#include <algorithm>

AvoidanceNode::AvoidanceNode() : Node("avoidance_node")
{
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "rear_lidar", 10, std::bind(&AvoidanceNode::lidar_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vector_markers", 10);
}

void AvoidanceNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto repulsion = calculate_repulsion(msg);
  auto attraction = calculate_attraction();
  auto resultant = calculate_resultant(attraction, repulsion);

  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = resultant[0];
  cmd_vel_msg.angular.z = resultant[1];

  cmd_vel_pub_->publish(cmd_vel_msg);
  publish_markers(attraction, repulsion, resultant);
}

std::vector<float> AvoidanceNode::calculate_repulsion(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float repulsion_x = 0.0;
  float repulsion_y = 0.0;
  float repulsion_gain = 0.5;

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];
    if (range < 1.0) // Threshold distance for obstacle
    {
      repulsion_x += repulsion_gain * (1.0 / range) * std::cos(angle);
      repulsion_y += repulsion_gain * (1.0 / range) * std::sin(angle);
    }
  }

  return {repulsion_x, repulsion_y};
}

std::vector<float> AvoidanceNode::calculate_attraction()
{
  float attraction_x = 1.0; // Constant forward attraction
  float attraction_y = 0.0;

  return {attraction_x, attraction_y};
}

std::vector<float> AvoidanceNode::calculate_resultant(const std::vector<float>& attraction, const std::vector<float>& repulsion)
{
  float resultant_x = attraction[0] + repulsion[0];
  float resultant_y = attraction[1] + repulsion[1];

  float linear_velocity = std::sqrt(resultant_x * resultant_x + resultant_y * resultant_y);
  float angular_velocity = std::atan2(resultant_y, resultant_x);

  return {linear_velocity, angular_velocity};
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
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
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

  marker_pub_->publish(create_marker(attraction, "attraction", 0, 0.0, 1.0, 0.0));
  marker_pub_->publish(create_marker(repulsion, "repulsion", 1, 1.0, 0.0, 0.0));
  marker_pub_->publish(create_marker(resultant, "resultant", 2, 0.0, 0.0, 1.0));
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}

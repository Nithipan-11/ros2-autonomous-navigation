#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  const double lookahead = this->declare_parameter<double>("lookahead_distance", 1.0);
  const double goal_tol = this->declare_parameter<double>("goal_tolerance", 0.3);
  const double speed = this->declare_parameter<double>("linear_speed", 0.5);
  const double max_ang = this->declare_parameter<double>("max_angular_speed", 1.0);
  const double rot_angle = this->declare_parameter<double>("rotate_in_place_angle", 1.0);
  const double base_offset = this->declare_parameter<double>("base_offset", 0.0);
  control_.setParams(lookahead, goal_tol, speed, max_ang, rot_angle, base_offset);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 10 Hz control loop
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop()
{
  if (!current_path_ || !robot_odom_) return;

  const auto cmd = control_.computeCommand(*current_path_, *robot_odom_);
  const bool moving = cmd.linear.x != 0.0 || cmd.angular.z != 0.0;

  // Only publish zeros once when stopping, so you can still teleop the robot when idle
  if (moving || was_moving_) {
    cmd_vel_pub_->publish(cmd);
  }
  was_moving_ = moving;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

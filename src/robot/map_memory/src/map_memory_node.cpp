#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const double size_m = this->declare_parameter<double>("size_m", 32.0);
  const std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);
  const int period_ms = this->declare_parameter<int>("update_period_ms", 1000);

  map_memory_.initMap(resolution, size_m, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&MapMemoryNode::updateMap, this));

  // Publish the (empty) map right away so the planner has something to plan on
  auto map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Quaternion -> yaw
  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;
}

void MapMemoryNode::updateMap()
{
  if (have_costmap_ && have_odom_) {
    const double moved = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);

    // Always fuse the very first costmap, then only after moving distance_threshold_
    if (!first_update_done_ || moved >= distance_threshold_) {
      map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
      last_x_ = robot_x_;
      last_y_ = robot_y_;
      first_update_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Map updated at (%.2f, %.2f)", robot_x_, robot_y_);
    }
  }

  // Re-publish every tick so late subscribers (planner, Foxglove) always get a map
  auto map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

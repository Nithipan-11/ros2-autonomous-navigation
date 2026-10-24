#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const double size_m = this->declare_parameter<double>("size_m", 30.0);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.0);
  const int max_cost = this->declare_parameter<int>("max_cost", 100);

  costmap_.initCostmap(resolution, size_m, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  costmap_.updateFromScan(*scan);

  auto msg = costmap_.getCostmap();
  msg.header.stamp = scan->header.stamp;
  msg.header.frame_id = scan->header.frame_id;   // grid lives in the lidar frame
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

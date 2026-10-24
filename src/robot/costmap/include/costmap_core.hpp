#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

// Builds a robot-centred costmap (in the lidar frame) from a single LaserScan.
// Cell values: -1 = unknown, 0 = free, 1..99 = inflated cost, 100 = obstacle.
class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap(double resolution, double size_m, double inflation_radius, int max_cost);

    // Rebuilds the whole grid from one scan
    void updateFromScan(const sensor_msgs::msg::LaserScan& scan);

    // Returns the grid as an OccupancyGrid (header must be filled by the caller)
    nav_msgs::msg::OccupancyGrid getCostmap() const;

  private:
    bool worldToGrid(double x, double y, int& gx, int& gy) const;
    void inflateObstacles(const std::vector<std::pair<int, int>>& obstacles);

    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 0;
    int height_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    std::vector<int8_t> grid_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_

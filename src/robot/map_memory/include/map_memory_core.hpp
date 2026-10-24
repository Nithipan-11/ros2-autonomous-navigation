#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Holds the global map (sim_world frame) and fuses robot-centred costmaps into it.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initMap(double resolution, double size_m, const std::string& frame_id);

    // Fuse a costmap taken at robot pose (x, y, yaw) into the global map.
    // Known cells (>= 0) overwrite, unknown cells (-1) keep the old value.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& getMap() const { return global_map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_

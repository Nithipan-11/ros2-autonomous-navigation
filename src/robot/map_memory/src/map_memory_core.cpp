#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution, double size_m, const std::string& frame_id)
{
  const int cells = static_cast<int>(std::round(size_m / resolution));
  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = resolution;
  global_map_.info.width = cells;
  global_map_.info.height = cells;
  // Centre the map on the world origin (the walls sit at +/-15 m)
  global_map_.info.origin.position.x = -size_m / 2.0;
  global_map_.info.origin.position.y = -size_m / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(cells * cells, -1);

  RCLCPP_INFO(logger_, "Global map: %dx%d cells @ %.2f m in frame '%s'",
              cells, cells, resolution, frame_id.c_str());
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw)
{
  const auto& g = global_map_.info;
  const auto& l = costmap.info;
  const double gres = g.resolution;
  const double lres = l.resolution;

  // Only visit global cells that the (rotated) costmap can possibly cover.
  // Costmap corners are at most this far from the robot:
  const double half_diag = 0.5 * std::hypot(l.width * lres, l.height * lres)
                         + std::hypot(l.origin.position.x + l.width * lres / 2.0,
                                      l.origin.position.y + l.height * lres / 2.0);

  auto clampi = [](int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); };
  const int gx_min = clampi(static_cast<int>(std::floor((robot_x - half_diag - g.origin.position.x) / gres)), 0, g.width - 1);
  const int gx_max = clampi(static_cast<int>(std::floor((robot_x + half_diag - g.origin.position.x) / gres)), 0, g.width - 1);
  const int gy_min = clampi(static_cast<int>(std::floor((robot_y - half_diag - g.origin.position.y) / gres)), 0, g.height - 1);
  const int gy_max = clampi(static_cast<int>(std::floor((robot_y + half_diag - g.origin.position.y) / gres)), 0, g.height - 1);

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);

  // Inverse mapping: for each global cell, look up which costmap cell covers it.
  // This never leaves holes, even when the robot is rotated.
  for (int gy = gy_min; gy <= gy_max; ++gy) {
    for (int gx = gx_min; gx <= gx_max; ++gx) {
      // Global cell centre in world coords
      const double wx = g.origin.position.x + (gx + 0.5) * gres;
      const double wy = g.origin.position.y + (gy + 0.5) * gres;

      // World -> robot (lidar) frame: rotate by -yaw about the robot
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double lx =  c * dx + s * dy;
      const double ly = -s * dx + c * dy;

      const int cx = static_cast<int>(std::floor((lx - l.origin.position.x) / lres));
      const int cy = static_cast<int>(std::floor((ly - l.origin.position.y) / lres));
      if (cx < 0 || cx >= static_cast<int>(l.width) || cy < 0 || cy >= static_cast<int>(l.height)) {
        continue;
      }

      const int8_t val = costmap.data[cy * l.width + cx];
      if (val >= 0) {   // linear fusion: known overwrites, unknown keeps old
        global_map_.data[gy * g.width + gx] = val;
      }
    }
  }
}

}  // namespace robot

#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initCostmap(double resolution, double size_m, double inflation_radius, int max_cost)
{
  resolution_ = resolution;
  width_ = static_cast<int>(std::round(size_m / resolution));
  height_ = width_;
  // Robot (lidar) sits at the centre of the grid
  origin_x_ = -size_m / 2.0;
  origin_y_ = -size_m / 2.0;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  grid_.assign(width_ * height_, -1);

  RCLCPP_INFO(logger_, "Costmap: %dx%d cells @ %.2f m, inflation %.2f m",
              width_, height_, resolution_, inflation_radius_);
}

bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const
{
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan)
{
  // Step 1: reset to unknown
  std::fill(grid_.begin(), grid_.end(), -1);

  const double max_grid_range = (width_ / 2.0) * resolution_;
  std::vector<std::pair<int, int>> obstacles;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double range = scan.ranges[i];

    // A hit is a finite reading inside the sensor limits.
    // inf / >= range_max means "nothing hit", so the beam is free up to range_max.
    if (std::isnan(range) || range < scan.range_min) {
      continue;  // no information from this beam
    }
    const bool hit = std::isfinite(range) && range < scan.range_max;
    const double free_len = std::min(hit ? range : scan.range_max, max_grid_range);

    // Step 2: raytrace free space along the beam
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    for (double d = 0.0; d < free_len - resolution_; d += resolution_ * 0.5) {
      int gx, gy;
      if (worldToGrid(d * c, d * s, gx, gy) && grid_[gy * width_ + gx] == -1) {
        grid_[gy * width_ + gx] = 0;
      }
    }

    // Step 3: record the obstacle cell (polar -> cartesian -> grid)
    if (hit) {
      int gx, gy;
      if (worldToGrid(range * c, range * s, gx, gy)) {
        obstacles.emplace_back(gx, gy);
      }
    }
  }

  // Step 4: mark obstacles (done after raytracing so they always win)
  for (const auto& [gx, gy] : obstacles) {
    grid_[gy * width_ + gx] = 100;
  }

  // Step 5: inflate
  inflateObstacles(obstacles);
}

void CostmapCore::inflateObstacles(const std::vector<std::pair<int, int>>& obstacles)
{
  const int r_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        const int x = ox + dx;
        const int y = oy + dy;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;

        const double dist = std::hypot(dx, dy) * resolution_;
        if (dist > inflation_radius_) continue;

        // cost = max_cost * (1 - distance / inflation_radius)
        const int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
        int8_t& cell = grid_[y * width_ + x];
        if (cost > cell) {   // unknown (-1) and free (0) are both lower than any cost
          cell = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getCostmap() const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;   // already row-major: index = y * width + x
  return msg;
}

}  // namespace robot

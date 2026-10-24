#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return (x != other.x || y != other.y); }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Min-heap by f_score
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) { return a.f_score > b.f_score; }
};

// ------------------- Planner -------------------

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void setParams(int lethal_cost, double cost_weight);

    // Runs A* on `map` from (sx, sy) to (gx, gy) in world coords.
    // Returns false if no path exists. `path` poses are in the map frame.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double sx, double sy, double gx, double gy,
                  nav_msgs::msg::Path& path) const;

  private:
    bool worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& c) const;

    rclcpp::Logger logger_;
    int lethal_cost_ = 60;      // cells at or above this are impassable
    double cost_weight_ = 5.0;  // how strongly to avoid inflated cells
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_

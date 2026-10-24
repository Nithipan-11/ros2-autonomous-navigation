#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::setParams(int lethal_cost, double cost_weight)
{
  lethal_cost_ = lethal_cost;
  cost_weight_ = cost_weight;
}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& c) const
{
  c.x = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
  c.y = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
  return c.x >= 0 && c.x < static_cast<int>(map.info.width) &&
         c.y >= 0 && c.y < static_cast<int>(map.info.height);
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           double sx, double sy, double gx, double gy,
                           nav_msgs::msg::Path& path) const
{
  path.poses.clear();

  CellIndex start, goal;
  if (!worldToCell(map, sx, sy, start) || !worldToCell(map, gx, gy, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is outside the map");
    return false;
  }

  const int W = map.info.width;
  const int H = map.info.height;
  auto idx = [W](const CellIndex& c) { return c.y * W + c.x; };

  // Unknown (-1) is treated as free: we plan optimistically and replan as the map fills in
  auto cellCost = [&](const CellIndex& c) {
    const int v = map.data[idx(c)];
    return v < 0 ? 0 : v;
  };

  // Octile distance: admissible heuristic for 8-connected grids
  auto heuristic = [&](const CellIndex& a) {
    const double dx = std::abs(a.x - goal.x);
    const double dy = std::abs(a.y - goal.y);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
  };

  std::vector<double> g_score(W * H, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(W * H, -1);
  std::vector<bool> closed(W * H, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  g_score[idx(start)] = 0.0;
  open.emplace(start, heuristic(start));

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    const CellIndex cur = open.top().index;
    open.pop();

    if (closed[idx(cur)]) continue;   // stale queue entry
    closed[idx(cur)] = true;

    if (cur == goal) { found = true; break; }

    for (int k = 0; k < 8; ++k) {
      const CellIndex nb(cur.x + dxs[k], cur.y + dys[k]);
      if (nb.x < 0 || nb.x >= W || nb.y < 0 || nb.y >= H) continue;
      if (closed[idx(nb)]) continue;

      // Impassable unless it's the goal (lets you click right next to a wall),
      // or we're escaping from an inflated start cell (never through a hard obstacle)
      const int c = cellCost(nb);
      const bool escaping = cellCost(cur) >= lethal_cost_ && c < 100;
      if (c >= lethal_cost_ && nb != goal && !escaping) continue;

      const double step = (k < 4) ? 1.0 : std::sqrt(2.0);
      // g(n): distance travelled, penalised by how close we pass to obstacles
      const double tentative = g_score[idx(cur)] + step * (1.0 + cost_weight_ * c / 100.0);

      if (tentative < g_score[idx(nb)]) {
        g_score[idx(nb)] = tentative;
        came_from[idx(nb)] = idx(cur);
        open.emplace(nb, tentative + heuristic(nb));   // f = g + h
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A*: no path found");
    return false;
  }

  // Walk back from goal to start
  std::vector<int> cells;
  for (int i = idx(goal); i != -1; i = came_from[i]) {
    cells.push_back(i);
  }
  std::reverse(cells.begin(), cells.end());

  const double res = map.info.resolution;
  for (int i : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = map.info.origin.position.x + (i % W + 0.5) * res;
    p.pose.position.y = map.info.origin.position.y + (i / W + 0.5) * res;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return true;
}

}  // namespace robot

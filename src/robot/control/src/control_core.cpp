#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::setParams(double lookahead, double goal_tolerance, double linear_speed,
                            double max_angular_speed, double rotate_in_place_angle, double base_offset)
{
  base_offset_ = base_offset;
  lookahead_distance_ = lookahead;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
  rotate_in_place_angle_ = rotate_in_place_angle;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot) const
{
  if (path.poses.empty()) return std::nullopt;

  // 1. Closest point on the path, so we never chase points behind us
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = computeDistance(path.poses[i].pose.position, robot);
    if (d < best) { best = d; closest = i; }
  }

  // 2. First point after it that is at least one lookahead distance away
  for (size_t i = closest; i < path.poses.size(); ++i) {
    if (computeDistance(path.poses[i].pose.position, robot) >= lookahead_distance_) {
      return path.poses[i];
    }
  }

  // 3. Near the end of the path: aim at the final point
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                                      const nav_msgs::msg::Odometry& odom) const
{
  geometry_msgs::msg::Twist cmd;   // zero by default = stop
  if (path.poses.empty()) return cmd;

  // Track the rear axle (the point the robot rotates about), not the lidar mounted ahead of it.
  // This is the standard pure-pursuit reference point and stops the body cutting corners.
  const double yaw = extractYaw(odom.pose.pose.orientation);
  geometry_msgs::msg::Point robot = odom.pose.pose.position;
  robot.x -= base_offset_ * std::cos(yaw);
  robot.y -= base_offset_ * std::sin(yaw);

  // Stop when the final waypoint is within tolerance
  if (computeDistance(path.poses.back().pose.position, robot) < goal_tolerance_) return cmd;

  const auto target = findLookaheadPoint(path, robot);
  if (!target) return cmd;

  // Angle between the robot's heading and the line to the lookahead point
  const double dx = target->pose.position.x - robot.x;
  const double dy = target->pose.position.y - robot.y;
  const double alpha = std::atan2(std::sin(std::atan2(dy, dx) - yaw),
                                  std::cos(std::atan2(dy, dx) - yaw));   // wrapped to [-pi, pi]

  // Target is far off to the side or behind: rotate on the spot first
  if (std::abs(alpha) > rotate_in_place_angle_) {
    cmd.angular.z = std::copysign(max_angular_speed_, alpha);
    return cmd;
  }

  // Pure pursuit: curvature of the arc through the lookahead point
  const double Ld = std::max(std::hypot(dx, dy), 1e-3);
  const double curvature = 2.0 * std::sin(alpha) / Ld;

  cmd.linear.x = linear_speed_;
  cmd.angular.z = std::clamp(linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

}  // namespace robot

#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

// Pure Pursuit path follower
class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void setParams(double lookahead, double goal_tolerance, double linear_speed,
                   double max_angular_speed, double rotate_in_place_angle, double base_offset);

    // One control step: returns the Twist to publish (zero Twist = stop)
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path& path,
                                             const nav_msgs::msg::Odometry& odom) const;

  private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot) const;

    static double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    static double extractYaw(const geometry_msgs::msg::Quaternion& q);

    rclcpp::Logger logger_;
    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.3;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
    double rotate_in_place_angle_ = 1.0;  // rad; above this, turn on the spot first
    double base_offset_ = 0.0;            // m from the rear axle (rotation centre) forward to the lidar
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_

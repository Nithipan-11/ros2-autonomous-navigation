#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <deque>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    struct Waypoint { double x; double y; };

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void startCourseCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void timerCallback();

    // Mission logic
    void loadCourse();
    void startNextGoal();
    void finishMission(bool success);
    bool goalReached() const;
    void planPath();
    void publishEmptyPath();
    void publishMarkers();

    robot::PlannerCore planner_;
    State state_ = State::WAITING_FOR_GOAL;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_course_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::Pose robot_pose_;   // lidar pose from odometry
    double base_x_ = 0.0, base_y_ = 0.0;   // rotation centre (rear axle) = lidar - base_offset * heading
    bool have_map_ = false;
    bool have_odom_ = false;

    // Mission state
    std::deque<Waypoint> queue_;        // goals still to visit after the current one
    Waypoint goal_{0.0, 0.0};           // current goal
    std::vector<Waypoint> visited_;     // goals already reached (for markers)
    std::vector<double> course_;        // flat [x1, y1, x2, y2, ...] from params
    bool auto_start_ = false;
    bool auto_started_ = false;
    int leg_ = 0;
    rclcpp::Time mission_start_;
    rclcpp::Time leg_start_;
    double distance_travelled_ = 0.0;
    double last_x_ = 0.0, last_y_ = 0.0;

    // Params
    double goal_tolerance_ = 0.5;       // final goal
    double waypoint_tolerance_ = 1.0;   // intermediate checkpoints: pass through without stopping
    double timeout_s_ = 90.0;           // per leg
    double base_offset_ = 0.0;          // distance from the rear axle forward to the lidar
};

#endif  // PLANNER_NODE_HPP_

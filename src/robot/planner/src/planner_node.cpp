#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  waypoint_tolerance_ = this->declare_parameter<double>("waypoint_tolerance", 1.0);
  timeout_s_ = this->declare_parameter<double>("timeout_s", 90.0);
  base_offset_ = this->declare_parameter<double>("base_offset", 0.0);
  const int lethal_cost = this->declare_parameter<int>("lethal_cost", 60);
  const double cost_weight = this->declare_parameter<double>("cost_weight", 5.0);
  course_ = this->declare_parameter<std::vector<double>>("course_waypoints", std::vector<double>{});
  auto_start_ = this->declare_parameter<bool>("auto_start_course", false);
  planner_.setParams(lethal_cost, cost_weight);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  start_course_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/start_course", 10, std::bind(&PlannerNode::startCourseCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  if (course_.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(), "course_waypoints must have an even length (x, y pairs)");
    course_.clear();
  }
  RCLCPP_INFO(this->get_logger(), "Planner ready: %zu course checkpoints, auto_start=%s",
              course_.size() / 2, auto_start_ ? "true" : "false");
}

// ------------------------------------------------------------------ callbacks

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  have_map_ = true;
  // New map info -> replan so the path routes around newly seen obstacles
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Clicked goals are appended to the queue, so you can click out a multi-stop route
  queue_.push_back({msg->point.x, msg->point.y});
  RCLCPP_INFO(this->get_logger(), "Queued goal (%.2f, %.2f) - %zu waiting",
              msg->point.x, msg->point.y, queue_.size());
  if (state_ == State::WAITING_FOR_GOAL) {
    visited_.clear();
    leg_ = 0;
    distance_travelled_ = 0.0;
    mission_start_ = this->now();
    startNextGoal();
  }
  publishMarkers();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Odometry gives the lidar pose. Plan for the robot's rotation centre (rear axle) instead:
  // the robot pivots there, so a path that keeps the axle clear keeps the whole body clear.
  const auto& q = msg->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double x = msg->pose.pose.position.x - base_offset_ * std::cos(yaw);
  const double y = msg->pose.pose.position.y - base_offset_ * std::sin(yaw);
  base_x_ = x;
  base_y_ = y;
  if (have_odom_ && state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    distance_travelled_ += std::hypot(x - last_x_, y - last_y_);
  }
  last_x_ = x;
  last_y_ = y;
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::startCourseCallback(const std_msgs::msg::Empty::SharedPtr)
{
  loadCourse();
}

void PlannerNode::timerCallback()
{
  // Auto-start the course once we know where we are and have a map
  if (auto_start_ && !auto_started_ && have_odom_ && have_map_) {
    auto_started_ = true;
    loadCourse();
  }

  publishMarkers();
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    const double leg_time = (this->now() - leg_start_).seconds();
    RCLCPP_INFO(this->get_logger(), "Checkpoint %d reached (%.2f, %.2f) in %.1f s",
                leg_, goal_.x, goal_.y, leg_time);
    visited_.push_back(goal_);
    if (queue_.empty()) {
      finishMission(true);
    } else {
      startNextGoal();
    }
    return;
  }

  if ((this->now() - leg_start_).seconds() > timeout_s_) {
    RCLCPP_WARN(this->get_logger(), "Checkpoint %d timed out after %.0f s, aborting", leg_, timeout_s_);
    finishMission(false);
  }
}

// ------------------------------------------------------------------ mission logic

void PlannerNode::loadCourse()
{
  if (course_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No course_waypoints set in params.yaml");
    return;
  }
  queue_.clear();
  visited_.clear();
  for (size_t i = 0; i + 1 < course_.size(); i += 2) {
    queue_.push_back({course_[i], course_[i + 1]});
  }
  leg_ = 0;
  distance_travelled_ = 0.0;
  mission_start_ = this->now();
  RCLCPP_INFO(this->get_logger(), "=== Course started: %zu checkpoints ===", queue_.size());
  startNextGoal();
}

void PlannerNode::startNextGoal()
{
  goal_ = queue_.front();
  queue_.pop_front();
  ++leg_;
  leg_start_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Heading to checkpoint %d: (%.2f, %.2f)", leg_, goal_.x, goal_.y);
  planPath();
}

void PlannerNode::finishMission(bool success)
{
  const double total = (this->now() - mission_start_).seconds();
  if (success) {
    RCLCPP_INFO(this->get_logger(), "=== FINISHED: %d checkpoints in %.1f s, %.1f m travelled ===",
                leg_, total, distance_travelled_);
  } else {
    RCLCPP_WARN(this->get_logger(), "=== ABORTED at checkpoint %d after %.1f s ===", leg_, total);
    queue_.clear();
  }
  state_ = State::WAITING_FOR_GOAL;
  publishEmptyPath();   // tells control to stop
}

bool PlannerNode::goalReached() const
{
  // Pass through intermediate checkpoints with a looser tolerance so the robot doesn't stop at each one
  const double tol = queue_.empty() ? goal_tolerance_ : waypoint_tolerance_;
  const double dx = goal_.x - base_x_;
  const double dy = goal_.y - base_y_;
  return std::sqrt(dx * dx + dy * dy) < tol;
}

void PlannerNode::planPath()
{
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or odometry");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;

  if (planner_.planPath(current_map_, base_x_, base_y_, goal_.x, goal_.y, path)) {
    path_pub_->publish(path);
  } else {
    publishEmptyPath();
  }
}

void PlannerNode::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
  path_pub_->publish(path);
}

void PlannerNode::publishMarkers()
{
  // Spheres + numbers for the route: green = done, yellow = current, blue = queued
  visualization_msgs::msg::MarkerArray arr;
  const std::string frame = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  std::vector<std::pair<Waypoint, int>> all;   // (waypoint, status: 0 done, 1 current, 2 queued)
  for (const auto& w : visited_) all.push_back({w, 0});
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) all.push_back({goal_, 1});
  for (const auto& w : queue_) all.push_back({w, 2});

  int id = 0;
  for (size_t i = 0; i < all.size(); ++i) {
    const auto& [w, status] = all[i];
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = this->now();
    m.ns = "waypoints";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position.x = w.x;
    m.pose.position.y = w.y;
    m.pose.position.z = 0.3;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color.a = 0.9;
    if (status == 0)      { m.color.g = 0.9; }
    else if (status == 1) { m.color.r = 1.0; m.color.g = 0.85; }
    else                  { m.color.b = 1.0; m.color.g = 0.4; }
    arr.markers.push_back(m);

    visualization_msgs::msg::Marker t = m;
    t.id = id++;
    t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.text = std::to_string(i + 1);
    t.pose.position.z = 1.2;
    t.scale.x = t.scale.y = 0.0;
    t.scale.z = 0.8;
    t.color.r = t.color.g = t.color.b = 1.0;
    arr.markers.push_back(t);
  }
  marker_pub_->publish(arr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

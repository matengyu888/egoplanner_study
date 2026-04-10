#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <regex>
#include <string>
#include <vector>
#include <sstream>

namespace
{

struct RouteGoal
{
  int order{0};
  int track_index{0};
  std::string kind;
  std::string name;
  double x{0.0};
  double y{0.0};
  double z_up{0.0};
  double yaw_rad{0.0};
};

struct FrameAnchor
{
  int order{0};
  int track_index{0};
  std::string name;
  double x{0.0};
  double y{0.0};
  double z_up{0.0};
  double yaw_rad{0.0};
  double nearest_track_dist{0.0};
};

class RmuaRouteGoalPublisher
{
public:
  explicit RmuaRouteGoalPublisher(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("route_file", route_file_, "");
    nh_.param<std::string>("odom_topic", odom_topic_, "/rmua/odom");
    nh_.param("reach_threshold", reach_threshold_, 4.0);
    nh_.param("publish_hz", publish_hz_, 1.0);
    nh_.param("start_order", start_order_, 1);
    nh_.param("reach_use_2d", reach_use_2d_, true);
    nh_.param("align_initial_altitude", align_initial_altitude_, true);
    nh_.param("start_from_nearest", start_from_nearest_, true);
    nh_.param("progress_window_points", progress_window_points_, 40);
    nh_.param("progress_snap_dist", progress_snap_dist_, 8.0);
    nh_.param("publish_lookahead_distance", publish_lookahead_distance_, 0.0);
    nh_.param("publish_direct_waypoint", publish_direct_waypoint_, true);
    nh_.param("direct_waypoint_window_points", direct_waypoint_window_points_, 1);
    nh_.param("min_publish_goal_distance", min_publish_goal_distance_, 3.0);
    nh_.param("passed_goal_projection_threshold", passed_goal_projection_threshold_, 0.2);
    nh_.param("passed_goal_lateral_threshold", passed_goal_lateral_threshold_, 2.5);
    nh_.param("turn_guard_angle_deg", turn_guard_angle_deg_, 75.0);
    nh_.param("turn_guard_distance", turn_guard_distance_, 10.0);
    nh_.param("same_goal_republish_period", same_goal_republish_period_, 0.5);
    nh_.param("same_goal_republish_dist", same_goal_republish_dist_, 2.0);
    nh_.param<std::string>("scoring_frames_file", scoring_frames_file_, "");
    nh_.param("frame_max_track_dist", frame_max_track_dist_, 2.0);
    nh_.param("frame_target_window_points", frame_target_window_points_, 30);
    nh_.param("frame_blend_points_before", frame_blend_points_before_, 40);
    nh_.param("frame_blend_points_after", frame_blend_points_after_, 8);
    nh_.param("frame_max_xy_offset", frame_max_xy_offset_, 0.6);
    nh_.param("frame_max_z_offset_up", frame_max_z_offset_up_, 1.8);
    nh_.param("frame_max_z_offset_down", frame_max_z_offset_down_, 0.0);
    nh_.param("frame_z_blend_points_before", frame_z_blend_points_before_, 90);
    nh_.param("frame_z_blend_points_after", frame_z_blend_points_after_, 16);

    if (!loadRoute())
    {
      ROS_FATAL("Failed to load RMUA route file: %s", route_file_.c_str());
      ros::shutdown();
      return;
    }
    loadFrameAnchors();

    int start_index = 0;
    while (start_index < static_cast<int>(goals_.size()) && goals_[start_index].order < start_order_)
      ++start_index;
    current_index_ = std::min(start_index, static_cast<int>(goals_.size()) - 1);

    odom_sub_ = nh_.subscribe(odom_topic_, 20, &RmuaRouteGoalPublisher::odomCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, true);
    direct_waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, publish_hz_)), &RmuaRouteGoalPublisher::timerCallback, this);

    ROS_INFO("Loaded %zu RMUA route goals from %s, starting from order %d (index %d).",
             goals_.size(), route_file_.c_str(), goals_[current_index_].order, current_index_);
  }

private:
  bool loadRoute()
  {
    if (route_file_.empty())
      return false;

    const bool is_csv = route_file_.size() >= 4 && route_file_.substr(route_file_.size() - 4) == ".csv";
    if (is_csv)
      return loadCsvRoute();

    std::ifstream fin(route_file_);
    if (!fin.is_open())
      return false;

    static const std::regex line_re(
        R"(^\s*(\d+)\s+track_idx=\s*\d+\s+dist=\s*[-0-9.]+\s+(\w+)\s+(\S+)\s+x=\s*([-0-9.]+)\s+y=\s*([-0-9.]+)\s+z=\s*([-0-9.]+)\s+yaw_deg=\s*([-0-9.]+))");

    std::string line;
    while (std::getline(fin, line))
    {
      std::smatch match;
      if (!std::regex_search(line, match, line_re))
        continue;

      RouteGoal goal;
      goal.order = std::stoi(match[1].str());
      goal.kind = match[2].str();
      goal.name = match[3].str();
      goal.x = std::stod(match[4].str());
      goal.y = std::stod(match[5].str());
      goal.z_up = -std::stod(match[6].str());
      goal.yaw_rad = std::stod(match[7].str()) * M_PI / 180.0;
      goals_.push_back(goal);
    }

    return !goals_.empty();
  }

  bool loadCsvRoute()
  {
    std::ifstream fin(route_file_);
    if (!fin.is_open())
      return false;

    std::string line;
    bool first_line = true;
    int order = 1;
    while (std::getline(fin, line))
    {
      if (line.empty())
        continue;
      if (first_line)
      {
        first_line = false;
        if (line.find("timestamp") != std::string::npos)
          continue;
      }

      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> cols;
      while (std::getline(ss, item, ','))
        cols.push_back(item);
      if (cols.size() < 8)
        continue;

      RouteGoal goal;
      goal.order = order++;
      goal.track_index = goal.order;
      goal.kind = "track";
      goal.name = "track_" + std::to_string(goal.order);
      goal.x = std::stod(cols[1]);
      goal.y = std::stod(cols[2]);
      goal.z_up = -std::stod(cols[3]);

      const double qx = std::stod(cols[4]);
      const double qy = std::stod(cols[5]);
      const double qz = std::stod(cols[6]);
      const double qw = std::stod(cols[7]);
      tf::Quaternion q(qx, qy, qz, qw);
      goal.yaw_rad = tf::getYaw(q);
      goals_.push_back(goal);
    }

    return !goals_.empty();
  }

  void loadFrameAnchors()
  {
    if (scoring_frames_file_.empty())
      return;

    std::ifstream fin(scoring_frames_file_);
    if (!fin.is_open())
    {
      ROS_WARN("Failed to open RMUA scoring frame file: %s", scoring_frames_file_.c_str());
      return;
    }

    static const std::regex line_re(
        R"(^\s*(\d+)\s+track_idx=\s*(\d+)\s+dist=\s*([-0-9.]+)\s+(\w+)\s+(\S+)\s+x=\s*([-0-9.]+)\s+y=\s*([-0-9.]+)\s+z=\s*([-0-9.]+)\s+yaw_deg=\s*([-0-9.]+))");

    std::string line;
    while (std::getline(fin, line))
    {
      std::smatch match;
      if (!std::regex_search(line, match, line_re))
        continue;

      if (match[4].str() != "frame")
        continue;

      const double nearest_dist = std::stod(match[3].str());
      if (nearest_dist > frame_max_track_dist_)
        continue;

      FrameAnchor anchor;
      anchor.order = std::stoi(match[1].str());
      anchor.track_index = std::stoi(match[2].str());
      anchor.name = match[5].str();
      anchor.x = std::stod(match[6].str());
      anchor.y = std::stod(match[7].str());
      anchor.z_up = -std::stod(match[8].str());
      anchor.yaw_rad = std::stod(match[9].str()) * M_PI / 180.0;
      anchor.nearest_track_dist = nearest_dist;
      frame_anchors_.push_back(anchor);
    }

    if (!frame_anchors_.empty())
    {
      ROS_INFO("Loaded %zu RMUA frame anchors from %s (max track dist %.2f m).",
               frame_anchors_.size(), scoring_frames_file_.c_str(), frame_max_track_dist_);
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.z = msg->pose.pose.position.z;
    have_odom_ = true;

    if (!route_index_initialized_)
    {
      route_index_initialized_ = true;
      if (start_from_nearest_)
      {
        const int nearest_index = findNearestGoalIndex();
        if (nearest_index >= 0)
        {
          current_index_ = nearest_index;
          last_reported_index_ = -1;
          ROS_INFO("RMUA route snapped to nearest target %03d %s at index %d.",
                   goals_[current_index_].order, goals_[current_index_].name.c_str(), current_index_);
        }
      }
    }

    if (!have_z_offset_ && align_initial_altitude_ && current_index_ < static_cast<int>(goals_.size()))
    {
      z_offset_ = current_pos_.z - goals_[current_index_].z_up;
      have_z_offset_ = true;
      ROS_INFO("Aligned RMUA route altitude offset to %.2f m.", z_offset_);
    }

    snapForwardByNearest();
    advanceIfReached();
  }

  int findNearestGoalIndex() const
  {
    if (!have_odom_ || goals_.empty())
      return -1;

    int nearest_index = -1;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(goals_.size()); ++i)
    {
      const auto& goal = goals_[i];
      const double dx = current_pos_.x - goal.x;
      const double dy = current_pos_.y - goal.y;
      const double goal_z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double dz = current_pos_.z - goal_z;
      const double dist = reach_use_2d_ ? std::sqrt(dx * dx + dy * dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < nearest_dist)
      {
        nearest_dist = dist;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  void advanceIfReached()
  {
    if (!have_odom_ || current_index_ >= static_cast<int>(goals_.size()))
      return;

    while (current_index_ < static_cast<int>(goals_.size()))
    {
      const auto& goal = goals_[current_index_];
      const double dx = current_pos_.x - goal.x;
      const double dy = current_pos_.y - goal.y;
      const double goal_z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double dz = current_pos_.z - goal_z;
      const double dist = reach_use_2d_ ? std::sqrt(dx * dx + dy * dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
      bool passed_goal = false;

      if (current_index_ + 1 < static_cast<int>(goals_.size()))
      {
        const auto& next_goal = goals_[current_index_ + 1];
        const double next_goal_z = next_goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
        const double seg_dx = next_goal.x - goal.x;
        const double seg_dy = next_goal.y - goal.y;
        const double seg_dz = reach_use_2d_ ? 0.0 : (next_goal_z - goal_z);
        const double seg_norm = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy + seg_dz * seg_dz);
        if (seg_norm > 1e-6)
        {
          const double along = (dx * seg_dx + dy * seg_dy + (reach_use_2d_ ? 0.0 : dz * seg_dz)) / seg_norm;
          const double along_dx = along * seg_dx / seg_norm;
          const double along_dy = along * seg_dy / seg_norm;
          const double along_dz = reach_use_2d_ ? 0.0 : along * seg_dz / seg_norm;
          const double lateral_dx = dx - along_dx;
          const double lateral_dy = dy - along_dy;
          const double lateral_dz = reach_use_2d_ ? 0.0 : (dz - along_dz);
          const double lateral = std::sqrt(lateral_dx * lateral_dx + lateral_dy * lateral_dy + lateral_dz * lateral_dz);
          passed_goal = along >= passed_goal_projection_threshold_ && lateral <= passed_goal_lateral_threshold_;
          if (passed_goal && dist > reach_threshold_)
          {
            ROS_INFO_THROTTLE(0.5,
                              "Advance RMUA route by passed-goal rule: idx=%d order=%03d dist=%.2f along=%.2f lateral=%.2f",
                              current_index_, goal.order, dist, along, lateral);
          }
        }
      }

      if (dist > reach_threshold_ && !passed_goal)
      {
        if (current_index_ != last_reported_index_)
        {
          ROS_INFO("RMUA route target %03d %s (%s), dist=%.2f", goal.order, goal.name.c_str(), goal.kind.c_str(), dist);
          last_reported_index_ = current_index_;
        }
        break;
      }

      ROS_INFO("Reached RMUA route target %03d %s, dist=%.2f", goal.order, goal.name.c_str(), dist);
      ++current_index_;
      last_reported_index_ = -1;
    }

    if (current_index_ >= static_cast<int>(goals_.size()))
      ROS_INFO_THROTTLE(2.0, "All RMUA route goals have been published and reached.");
  }

  void snapForwardByNearest()
  {
    if (!have_odom_ || current_index_ >= static_cast<int>(goals_.size()))
      return;

    int effective_window_points = progress_window_points_;
    if (effective_window_points <= 0 && publish_direct_waypoint_ && direct_waypoint_window_points_ > 1)
      effective_window_points = direct_waypoint_window_points_;
    if (effective_window_points <= 0)
      return;

    double effective_snap_dist = progress_snap_dist_;
    if (effective_snap_dist <= 1e-3 && publish_direct_waypoint_ && direct_waypoint_window_points_ > 1)
      effective_snap_dist = std::max(6.0, reach_threshold_ * 3.0);
    if (effective_snap_dist <= 1e-3)
      return;

    const int raw_window_end = std::min(static_cast<int>(goals_.size()) - 1, current_index_ + effective_window_points);
    const int window_end = applyTurnGuardToWindowEnd(current_index_, raw_window_end);
    int nearest_index = current_index_;
    double nearest_dist = std::numeric_limits<double>::infinity();

    for (int i = current_index_; i <= window_end; ++i)
    {
      const auto& goal = goals_[i];
      const double goal_z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double dx = current_pos_.x - goal.x;
      const double dy = current_pos_.y - goal.y;
      const double dz = current_pos_.z - goal_z;
      const double dist = reach_use_2d_ ? std::sqrt(dx * dx + dy * dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < nearest_dist)
      {
        nearest_dist = dist;
        nearest_index = i;
      }
    }

    if (nearest_index > current_index_ && nearest_dist <= effective_snap_dist)
    {
      ROS_INFO("RMUA route progress snapped from index %d to %d (order %03d), nearest_dist=%.2f window=%d snap=%.2f",
               current_index_, nearest_index, goals_[nearest_index].order, nearest_dist,
               effective_window_points, effective_snap_dist);
      current_index_ = nearest_index;
      last_reported_index_ = -1;
    }
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!have_odom_ || current_index_ >= static_cast<int>(goals_.size()))
      return;

    const int publish_index = getPublishIndex();
    const int window_points = std::max(1, direct_waypoint_window_points_);
    const int raw_window_end = std::min(static_cast<int>(goals_.size()) - 1, publish_index + window_points - 1);
    const int window_end = applyTurnGuardToWindowEnd(publish_index, raw_window_end);
    if (publish_direct_waypoint_)
    {
      const bool same_window = publish_index == last_published_index_ &&
                               current_index_ == last_published_current_index_ &&
                               window_end == last_published_window_end_;
      if (same_window)
      {
        if (same_goal_republish_period_ <= 1e-3)
          return;
        const double elapsed = (ros::Time::now() - last_published_time_).toSec();
        if (elapsed < same_goal_republish_period_)
          return;
      }
    }
    else if (publish_index == last_published_index_ && !shouldRepublishCurrentGoal())
    {
      return;
    }

    const auto& track_goal = goals_[publish_index];
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = track_goal.x;
    msg.pose.position.y = track_goal.y;
    msg.pose.position.z = track_goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(track_goal.yaw_rad);

    const FrameAnchor* anchor = getActiveFrameAnchor(publish_index);
    if (anchor != nullptr)
    {
      const double xy_weight = computeBlendWeight(
          publish_index, *anchor, frame_blend_points_before_, frame_blend_points_after_);
      const double z_weight = computeBlendWeight(
          publish_index, *anchor, frame_z_blend_points_before_, frame_z_blend_points_after_);
      const double track_z = track_goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double anchor_z = anchor->z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double dx = anchor->x - track_goal.x;
      const double dy = anchor->y - track_goal.y;
      const double xy_norm = std::sqrt(dx * dx + dy * dy);
      const double xy_scale = xy_norm > frame_max_xy_offset_ && xy_norm > 1e-6 ? frame_max_xy_offset_ / xy_norm : 1.0;
      const double dz_raw = anchor_z - track_z;
      const double dz = std::max(-frame_max_z_offset_down_, std::min(dz_raw, frame_max_z_offset_up_));
      msg.pose.position.x = track_goal.x + dx * xy_scale * xy_weight;
      msg.pose.position.y = track_goal.y + dy * xy_scale * xy_weight;
      msg.pose.position.z = track_z + dz * z_weight;
      if (active_anchor_order_ != anchor->order)
      {
        active_anchor_order_ = anchor->order;
        ROS_INFO("Blending toward RMUA frame %03d %s at track_idx=%d, nearest_track_dist=%.2f",
                 anchor->order, anchor->name.c_str(), anchor->track_index, anchor->nearest_track_dist);
      }
    }
    else
    {
      active_anchor_order_ = -1;
    }
    if (publish_direct_waypoint_)
    {
      nav_msgs::Path path_msg;
      path_msg.header = msg.header;
      path_msg.poses.push_back(msg);
      for (int future_index = publish_index + 1; future_index <= window_end; ++future_index)
      {
        const auto& future_goal = goals_[future_index];
        geometry_msgs::PoseStamped future_msg;
        future_msg.header = msg.header;
        future_msg.pose.position.x = future_goal.x;
        future_msg.pose.position.y = future_goal.y;
        future_msg.pose.position.z = future_goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
        future_msg.pose.orientation = tf::createQuaternionMsgFromYaw(future_goal.yaw_rad);
        path_msg.poses.push_back(future_msg);
      }
      direct_waypoint_pub_.publish(path_msg);
    }
    else
    {
      goal_pub_.publish(msg);
    }
    last_published_index_ = publish_index;
    last_published_current_index_ = current_index_;
    last_published_window_end_ = window_end;
    last_published_time_ = ros::Time::now();
    if (publish_direct_waypoint_)
    {
      ROS_INFO_THROTTLE(0.5, "Published RMUA direct waypoint current_index=%d publish_index=%d order=%03d pos=(%.2f, %.2f, %.2f)",
                        current_index_, publish_index, track_goal.order,
                        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
    else
    {
      ROS_INFO("Published RMUA goal current_index=%d publish_index=%d order=%03d pos=(%.2f, %.2f, %.2f)",
               current_index_, publish_index, track_goal.order,
               msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
  }

  bool shouldRepublishCurrentGoal() const
  {
    if (last_published_index_ < 0 || last_published_index_ >= static_cast<int>(goals_.size()))
      return true;

    if (same_goal_republish_period_ <= 1e-3)
      return false;

    const double elapsed = (ros::Time::now() - last_published_time_).toSec();
    if (elapsed < same_goal_republish_period_)
      return false;

    const auto& goal = goals_[last_published_index_];
    const double goal_z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
    const double dx = current_pos_.x - goal.x;
    const double dy = current_pos_.y - goal.y;
    const double dz = current_pos_.z - goal_z;
    const double dist = reach_use_2d_ ? std::sqrt(dx * dx + dy * dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist <= same_goal_republish_dist_;
  }

  const FrameAnchor* getActiveFrameAnchor(int publish_index) const
  {
    if (frame_anchors_.empty())
      return nullptr;

    const int window_end = std::min(static_cast<int>(goals_.size()) - 1, publish_index + frame_target_window_points_);
    for (const auto& anchor : frame_anchors_)
    {
      const int anchor_index = std::max(0, anchor.track_index - 1);
      if (anchor_index < current_index_)
        continue;
      if (anchor_index <= window_end)
        return &anchor;
    }
    return nullptr;
  }

  double computeBlendWeight(int publish_index, const FrameAnchor& anchor, int before_points, int after_points) const
  {
    const int anchor_index = std::max(0, anchor.track_index - 1);
    const int delta = anchor_index - publish_index;
    double raw_weight = 0.0;

    if (delta >= 0)
    {
      if (delta >= before_points)
        return 0.0;
      raw_weight = 1.0 - static_cast<double>(delta) / std::max(1, before_points);
    }
    else
    {
      const int passed_points = -delta;
      if (passed_points >= after_points)
        return 0.0;
      raw_weight = 1.0 - static_cast<double>(passed_points) / std::max(1, after_points);
    }

    raw_weight = std::max(0.0, std::min(raw_weight, 1.0));
    // Smoothstep avoids sharp target changes when entering/leaving a frame window.
    return raw_weight * raw_weight * (3.0 - 2.0 * raw_weight);
  }

  int getPublishIndex() const
  {
    if (publish_lookahead_distance_ <= 1e-3 || current_index_ >= static_cast<int>(goals_.size()) - 1)
      return ensureMinimumPublishDistance(current_index_);

    double accumulated_dist = 0.0;
    int publish_index = current_index_;
    while (publish_index + 1 < static_cast<int>(goals_.size()))
    {
      const auto& from = goals_[publish_index];
      const auto& to = goals_[publish_index + 1];
      const double dz = (to.z_up + (have_z_offset_ ? z_offset_ : 0.0)) -
                        (from.z_up + (have_z_offset_ ? z_offset_ : 0.0));
      accumulated_dist += std::sqrt(
          (to.x - from.x) * (to.x - from.x) +
          (to.y - from.y) * (to.y - from.y) +
          dz * dz);
      ++publish_index;
      if (accumulated_dist >= publish_lookahead_distance_)
        break;
    }
    return ensureMinimumPublishDistance(publish_index);
  }

  int ensureMinimumPublishDistance(int publish_index) const
  {
    if (min_publish_goal_distance_ <= 1e-3)
      return publish_index;

    int adjusted_index = publish_index;
    while (adjusted_index + 1 < static_cast<int>(goals_.size()))
    {
      const auto& goal = goals_[adjusted_index];
      const double goal_z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
      const double dx = current_pos_.x - goal.x;
      const double dy = current_pos_.y - goal.y;
      const double dz = current_pos_.z - goal_z;
      const double dist = reach_use_2d_ ? std::sqrt(dx * dx + dy * dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist >= min_publish_goal_distance_)
        break;
      ++adjusted_index;
    }
    return adjusted_index;
  }

  double goalSegmentLength(int from_index, int to_index) const
  {
    const auto& from = goals_[from_index];
    const auto& to = goals_[to_index];
    const double dz = (reach_use_2d_ ? 0.0 : ((to.z_up + (have_z_offset_ ? z_offset_ : 0.0)) -
                                              (from.z_up + (have_z_offset_ ? z_offset_ : 0.0))));
    return std::sqrt((to.x - from.x) * (to.x - from.x) +
                     (to.y - from.y) * (to.y - from.y) +
                     dz * dz);
  }

  double turnAngleDegAt(int middle_index) const
  {
    if (middle_index <= 0 || middle_index >= static_cast<int>(goals_.size()) - 1)
      return 0.0;

    const auto& prev = goals_[middle_index - 1];
    const auto& curr = goals_[middle_index];
    const auto& next = goals_[middle_index + 1];

    const double u_dx = curr.x - prev.x;
    const double u_dy = curr.y - prev.y;
    const double u_dz = reach_use_2d_ ? 0.0 : ((curr.z_up + (have_z_offset_ ? z_offset_ : 0.0)) -
                                               (prev.z_up + (have_z_offset_ ? z_offset_ : 0.0)));
    const double v_dx = next.x - curr.x;
    const double v_dy = next.y - curr.y;
    const double v_dz = reach_use_2d_ ? 0.0 : ((next.z_up + (have_z_offset_ ? z_offset_ : 0.0)) -
                                               (curr.z_up + (have_z_offset_ ? z_offset_ : 0.0)));

    const double u_norm = std::sqrt(u_dx * u_dx + u_dy * u_dy + u_dz * u_dz);
    const double v_norm = std::sqrt(v_dx * v_dx + v_dy * v_dy + v_dz * v_dz);
    if (u_norm <= 1e-6 || v_norm <= 1e-6)
      return 0.0;

    double cos_angle = (u_dx * v_dx + u_dy * v_dy + u_dz * v_dz) / (u_norm * v_norm);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle) * 180.0 / M_PI;
  }

  int applyTurnGuardToWindowEnd(int start_index, int raw_window_end) const
  {
    if (turn_guard_angle_deg_ <= 1e-3 || raw_window_end <= start_index + 1)
      return raw_window_end;

    double distance_from_start = 0.0;
    for (int i = start_index + 1; i <= raw_window_end - 1; ++i)
    {
      distance_from_start += goalSegmentLength(i - 1, i);
      const double turn_angle_deg = turnAngleDegAt(i);
      if (turn_angle_deg >= turn_guard_angle_deg_ && distance_from_start > turn_guard_distance_)
      {
        const int guarded_end = std::max(start_index, i);
        ROS_INFO_THROTTLE(0.5,
                          "RMUA turn guard truncating preview: start=%d raw_end=%d guarded_end=%d turn_idx=%d angle=%.1f dist=%.1f",
                          start_index, raw_window_end, guarded_end, i, turn_angle_deg, distance_from_start);
        return guarded_end;
      }
    }

    return raw_window_end;
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher direct_waypoint_pub_;
  ros::Timer timer_;

  std::string route_file_;
  std::string odom_topic_;
  double reach_threshold_{4.0};
  double publish_hz_{1.0};
  int start_order_{1};
  int current_index_{0};
  int last_reported_index_{-1};
  bool have_odom_{false};
  bool reach_use_2d_{true};
  bool align_initial_altitude_{true};
  bool start_from_nearest_{true};
  int progress_window_points_{40};
  double progress_snap_dist_{8.0};
  bool publish_direct_waypoint_{true};
  int direct_waypoint_window_points_{1};
  bool route_index_initialized_{false};
  bool have_z_offset_{false};
  double z_offset_{0.0};
  double publish_lookahead_distance_{0.0};
  double min_publish_goal_distance_{3.0};
  double passed_goal_projection_threshold_{0.2};
  double passed_goal_lateral_threshold_{2.5};
  double turn_guard_angle_deg_{75.0};
  double turn_guard_distance_{10.0};
  double same_goal_republish_period_{0.5};
  double same_goal_republish_dist_{2.0};
  std::string scoring_frames_file_;
  double frame_max_track_dist_{2.0};
  int frame_target_window_points_{30};
  int frame_blend_points_before_{40};
  int frame_blend_points_after_{8};
  double frame_max_xy_offset_{0.6};
  double frame_max_z_offset_up_{1.8};
  double frame_max_z_offset_down_{0.0};
  int frame_z_blend_points_before_{90};
  int frame_z_blend_points_after_{16};
  int active_anchor_order_{-1};
  int last_published_index_{-1};
  int last_published_current_index_{-1};
  int last_published_window_end_{-1};
  ros::Time last_published_time_;
  geometry_msgs::Point current_pos_;
  std::vector<RouteGoal> goals_;
  std::vector<FrameAnchor> frame_anchors_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmua_route_goal_publisher");
  ros::NodeHandle nh("~");
  RmuaRouteGoalPublisher node(nh);
  ros::spin();
  return 0;
}

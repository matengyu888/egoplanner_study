#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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
  std::string kind;
  std::string name;
  double x{0.0};
  double y{0.0};
  double z_up{0.0};
  double yaw_rad{0.0};
};

class RmuaRouteGoalPublisher
{
public:
  explicit RmuaRouteGoalPublisher(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("route_file", route_file_, "");
    nh_.param("reach_threshold", reach_threshold_, 4.0);
    nh_.param("publish_hz", publish_hz_, 1.0);
    nh_.param("start_order", start_order_, 1);
    nh_.param("reach_use_2d", reach_use_2d_, true);
    nh_.param("align_initial_altitude", align_initial_altitude_, true);
    nh_.param("start_from_nearest", start_from_nearest_, true);
    nh_.param("publish_lookahead_distance", publish_lookahead_distance_, 0.0);

    if (!loadRoute())
    {
      ROS_FATAL("Failed to load RMUA route file: %s", route_file_.c_str());
      ros::shutdown();
      return;
    }

    int start_index = 0;
    while (start_index < static_cast<int>(goals_.size()) && goals_[start_index].order < start_order_)
      ++start_index;
    current_index_ = std::min(start_index, static_cast<int>(goals_.size()) - 1);

    odom_sub_ = nh_.subscribe("/rmua/odom", 20, &RmuaRouteGoalPublisher::odomCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, publish_hz_)), &RmuaRouteGoalPublisher::timerCallback, this);

    ROS_INFO("Loaded %zu RMUA route goals, starting from order %d.", goals_.size(), goals_[current_index_].order);
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

      if (dist > reach_threshold_)
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

  void timerCallback(const ros::TimerEvent&)
  {
    if (!have_odom_ || current_index_ >= static_cast<int>(goals_.size()))
      return;

    const int publish_index = getPublishIndex();
    const auto& goal = goals_[publish_index];
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = goal.x;
    msg.pose.position.y = goal.y;
    msg.pose.position.z = goal.z_up + (have_z_offset_ ? z_offset_ : 0.0);
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(goal.yaw_rad);
    goal_pub_.publish(msg);
  }

  int getPublishIndex() const
  {
    if (publish_lookahead_distance_ <= 1e-3 || current_index_ >= static_cast<int>(goals_.size()) - 1)
      return current_index_;

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
    return publish_index;
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Timer timer_;

  std::string route_file_;
  double reach_threshold_{4.0};
  double publish_hz_{1.0};
  int start_order_{1};
  int current_index_{0};
  int last_reported_index_{-1};
  bool have_odom_{false};
  bool reach_use_2d_{true};
  bool align_initial_altitude_{true};
  bool start_from_nearest_{true};
  bool route_index_initialized_{false};
  bool have_z_offset_{false};
  double z_offset_{0.0};
  double publish_lookahead_distance_{0.0};
  geometry_msgs::Point current_pos_;
  std::vector<RouteGoal> goals_;
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

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <airsim_ros/Takeoff.h>

#include <Eigen/Geometry>

namespace
{

Eigen::Vector3d simToPlanner(const Eigen::Vector3d& v)
{
  return Eigen::Vector3d(v.x(), v.y(), -v.z());
}

Eigen::Quaterniond simToPlanner(const Eigen::Quaterniond& q_sim)
{
  static const Eigen::DiagonalMatrix<double, 3> transform(1.0, 1.0, -1.0);
  Eigen::Matrix3d r_planner = transform * q_sim.normalized().toRotationMatrix() * transform;
  return Eigen::Quaterniond(r_planner).normalized();
}

class RmuaBridge
{
public:
  explicit RmuaBridge(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param("auto_takeoff", auto_takeoff_, true);
    nh_.param("goal_republish_hz", goal_republish_hz_, 1.0);
    nh_.param("forward_sim_goal", forward_sim_goal_, true);
    nh_.param("lidar_min_range", lidar_min_range_, 1.2);
    nh_.param("publish_lidar", publish_lidar_, true);

    pose_sub_ = nh_.subscribe("/airsim_node/drone_1/debug/pose_gt", 50, &RmuaBridge::poseCallback, this);
    init_pose_sub_ = nh_.subscribe("/airsim_node/initial_pose", 1, &RmuaBridge::initPoseCallback, this);
    if (forward_sim_goal_)
      goal_sub_ = nh_.subscribe("/airsim_node/end_goal", 1, &RmuaBridge::goalCallback, this);
    if (publish_lidar_)
      lidar_sub_ = nh_.subscribe("/airsim_node/drone_1/lidar", 5, &RmuaBridge::lidarCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/rmua/odom", 50);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/rmua/lidar", 5);

    takeoff_client_ = nh_.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    goal_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, goal_republish_hz_)), &RmuaBridge::goalTimerCallback, this);
  }

private:
  void initPoseCallback(const geometry_msgs::PoseStampedConstPtr&)
  {
    if (!auto_takeoff_ || takeoff_sent_)
      return;

    airsim_ros::Takeoff srv;
    srv.request.waitOnLastTask = true;
    if (takeoff_client_.call(srv))
    {
      takeoff_sent_ = true;
      ROS_INFO("RMUA takeoff service called.");
    }
    else
    {
      ROS_WARN_THROTTLE(2.0, "Failed to call RMUA takeoff service.");
    }
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    latest_goal_ = *msg;
    latest_goal_.header.frame_id = "world";
    latest_goal_.pose.position.x = msg->pose.position.x;
    latest_goal_.pose.position.y = msg->pose.position.y;
    latest_goal_.pose.position.z = -msg->pose.position.z;

    const Eigen::Quaterniond q_sim(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    const Eigen::Quaterniond q_planner = simToPlanner(q_sim);
    latest_goal_.pose.orientation.w = q_planner.w();
    latest_goal_.pose.orientation.x = q_planner.x();
    latest_goal_.pose.orientation.y = q_planner.y();
    latest_goal_.pose.orientation.z = q_planner.z();

    have_goal_ = true;
    goal_pub_.publish(latest_goal_);
  }

  void goalTimerCallback(const ros::TimerEvent&)
  {
    if (!have_goal_)
      return;

    latest_goal_.header.stamp = ros::Time::now();
    goal_pub_.publish(latest_goal_);
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const Eigen::Vector3d pos_sim(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const Eigen::Quaterniond q_sim(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    const Eigen::Vector3d pos = simToPlanner(pos_sim);
    const Eigen::Quaterniond q = simToPlanner(q_sim);
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    if (have_last_pose_)
    {
      const double dt = (stamp - last_stamp_).toSec();
      if (dt > 1e-3)
      {
        const Eigen::Vector3d vel = (pos - last_pos_) / dt;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        Eigen::Quaterniond dq = last_q_.inverse() * q;
        dq.normalize();
        Eigen::AngleAxisd aa(dq);
        const Eigen::Vector3d omega = aa.axis() * aa.angle() / dt;
        odom.twist.twist.angular.x = omega.x();
        odom.twist.twist.angular.y = omega.y();
        odom.twist.twist.angular.z = omega.z();
      }
    }

    last_pos_ = pos;
    last_q_ = q;
    last_stamp_ = stamp;
    have_last_pose_ = true;
    current_pos_ = pos;
    current_q_ = q;
    have_current_pose_ = true;
    ROS_INFO_THROTTLE(1.0, "RMUA bridge pose callback: sim=(%.2f, %.2f, %.2f) planner=(%.2f, %.2f, %.2f)",
                      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, pos.x(), pos.y(), pos.z());
    odom_pub_.publish(odom);
  }

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!have_current_pose_)
      return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.reserve(cloud.points.size());
    for (const auto& pt : cloud.points)
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, -pt.z);
      if (pt_sensor.norm() < lidar_min_range_)
        continue;
      const Eigen::Vector3d pt_world = current_q_ * pt_sensor + current_pos_;
      pcl::PointXYZ out_pt;
      out_pt.x = pt_world.x();
      out_pt.y = pt_world.y();
      out_pt.z = pt_world.z();
      filtered.push_back(out_pt);
    }
    filtered.width = filtered.points.size();
    filtered.height = 1;
    filtered.is_dense = true;

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(filtered, out);
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "world";
    lidar_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber lidar_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher lidar_pub_;
  ros::ServiceClient takeoff_client_;
  ros::Timer goal_timer_;

  bool auto_takeoff_{true};
  bool forward_sim_goal_{true};
  bool publish_lidar_{true};
  bool takeoff_sent_{false};
  bool have_goal_{false};
  bool have_last_pose_{false};
  bool have_current_pose_{false};
  double goal_republish_hz_{1.0};
  double lidar_min_range_{1.2};
  ros::Time last_stamp_;
  Eigen::Vector3d last_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d current_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond current_q_{Eigen::Quaterniond::Identity()};
  geometry_msgs::PoseStamped latest_goal_;
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmua_bridge");
  ros::NodeHandle nh("~");
  RmuaBridge bridge(nh);
  ros::spin();
  return 0;
}

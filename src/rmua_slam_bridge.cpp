#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

namespace
{

Eigen::Vector3d flipZ(const Eigen::Vector3d& v)
{
  return Eigen::Vector3d(v.x(), v.y(), -v.z());
}

Eigen::Quaterniond flipZ(const Eigen::Quaterniond& q_in)
{
  static const Eigen::DiagonalMatrix<double, 3> transform(1.0, 1.0, -1.0);
  const Eigen::Matrix3d rot = transform * q_in.normalized().toRotationMatrix() * transform;
  return Eigen::Quaterniond(rot).normalized();
}

geometry_msgs::Pose convertPose(const geometry_msgs::Pose& in)
{
  geometry_msgs::Pose out = in;
  const Eigen::Vector3d pos_in(in.position.x, in.position.y, in.position.z);
  const Eigen::Quaterniond q_in(in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z);

  const Eigen::Vector3d pos_out = flipZ(pos_in);
  const Eigen::Quaterniond q_out = flipZ(q_in);

  out.position.x = pos_out.x();
  out.position.y = pos_out.y();
  out.position.z = pos_out.z();
  out.orientation.w = q_out.w();
  out.orientation.x = q_out.x();
  out.orientation.y = q_out.y();
  out.orientation.z = q_out.z();
  return out;
}

class RmuaSlamBridge
{
public:
  explicit RmuaSlamBridge(ros::NodeHandle& nh)
  {
    nh.param<std::string>("pose_in", pose_in_, "/uav/state/pose");
    nh.param<std::string>("odom_in", odom_in_, "/uav/state/odom");
    nh.param<std::string>("path_in", path_in_, "/uav/state/path");
    nh.param<std::string>("scan_in", scan_in_, "/cloud_registered");
    nh.param<std::string>("map_in", map_in_, "/Laser_map");

    nh.param<std::string>("pose_out", pose_out_, "/rmua_slam/pose");
    nh.param<std::string>("odom_out", odom_out_, "/rmua_slam/odom");
    nh.param<std::string>("path_out", path_out_, "/rmua_slam/path");
    nh.param<std::string>("scan_out", scan_out_, "/rmua_slam/cloud_registered");
    nh.param<std::string>("map_out", map_out_, "/rmua_slam/laser_map");

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_out_, 20);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_out_, 20);
    path_pub_ = nh.advertise<nav_msgs::Path>(path_out_, 10);
    scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>(scan_out_, 5);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_out_, 2);

    pose_sub_ = nh.subscribe(pose_in_, 20, &RmuaSlamBridge::poseCallback, this);
    odom_sub_ = nh.subscribe(odom_in_, 20, &RmuaSlamBridge::odomCallback, this);
    path_sub_ = nh.subscribe(path_in_, 5, &RmuaSlamBridge::pathCallback, this);
    scan_sub_ = nh.subscribe(scan_in_, 5, &RmuaSlamBridge::scanCallback, this);
    map_sub_ = nh.subscribe(map_in_, 2, &RmuaSlamBridge::mapCallback, this);
  }

private:
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    geometry_msgs::PoseStamped out = *msg;
    out.pose = convertPose(msg->pose);
    pose_pub_.publish(out);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    nav_msgs::Odometry out = *msg;
    out.pose.pose = convertPose(msg->pose.pose);

    const Eigen::Vector3d linear_in(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const Eigen::Vector3d angular_in(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    const Eigen::Vector3d linear_out = flipZ(linear_in);
    const Eigen::Vector3d angular_out = flipZ(angular_in);

    out.twist.twist.linear.x = linear_out.x();
    out.twist.twist.linear.y = linear_out.y();
    out.twist.twist.linear.z = linear_out.z();
    out.twist.twist.angular.x = angular_out.x();
    out.twist.twist.angular.y = angular_out.y();
    out.twist.twist.angular.z = angular_out.z();
    odom_pub_.publish(out);
  }

  void pathCallback(const nav_msgs::PathConstPtr& msg)
  {
    nav_msgs::Path out = *msg;
    for (auto& pose_stamped : out.poses)
      pose_stamped.pose = convertPose(pose_stamped.pose);
    path_pub_.publish(out);
  }

  void publishCloud(const sensor_msgs::PointCloud2ConstPtr& msg, const ros::Publisher& pub)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    pcl::PointCloud<pcl::PointXYZI> cloud_out = cloud_in;
    for (auto& pt : cloud_out.points)
      pt.z = -pt.z;

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(cloud_out, out);
    out.header = msg->header;
    pub.publish(out);
  }

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    publishCloud(msg, scan_pub_);
  }

  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    publishCloud(msg, map_pub_);
  }

  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher map_pub_;

  std::string pose_in_;
  std::string odom_in_;
  std::string path_in_;
  std::string scan_in_;
  std::string map_in_;
  std::string pose_out_;
  std::string odom_out_;
  std::string path_out_;
  std::string scan_out_;
  std::string map_out_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmua_slam_bridge");
  ros::NodeHandle nh("~");
  RmuaSlamBridge bridge(nh);
  ros::spin();
  return 0;
}

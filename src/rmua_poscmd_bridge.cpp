#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

namespace
{

Eigen::Vector3d enuToNedPosition(const Eigen::Vector3d& p)
{
  return Eigen::Vector3d(p.x(), p.y(), -p.z());
}

double wrapAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

class RmuaPositionCommandBridge
{
public:
  explicit RmuaPositionCommandBridge(ros::NodeHandle& nh)
  {
    pos_cmd_sub_ = nh.subscribe("position_cmd_in", 50, &RmuaPositionCommandBridge::positionCmdCallback, this);
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 20);
  }

private:
  void positionCmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg)
  {
    quadrotor_msgs::PositionCommand out = *msg;

    const Eigen::Vector3d pos_enu(msg->position.x, msg->position.y, msg->position.z);
    const Eigen::Vector3d vel_enu(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    const Eigen::Vector3d acc_enu(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);

    const Eigen::Vector3d pos_ned = enuToNedPosition(pos_enu);
    const Eigen::Vector3d vel_ned = enuToNedPosition(vel_enu);
    const Eigen::Vector3d acc_ned = enuToNedPosition(acc_enu);

    out.position.x = pos_ned.x();
    out.position.y = pos_ned.y();
    out.position.z = pos_ned.z();

    out.velocity.x = vel_ned.x();
    out.velocity.y = vel_ned.y();
    out.velocity.z = vel_ned.z();

    out.acceleration.x = acc_ned.x();
    out.acceleration.y = acc_ned.y();
    out.acceleration.z = acc_ned.z();

    out.yaw = wrapAngle(msg->yaw);
    out.yaw_dot = msg->yaw_dot;

    pos_cmd_pub_.publish(out);
  }

  ros::Subscriber pos_cmd_sub_;
  ros::Publisher pos_cmd_pub_;
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmua_poscmd_bridge");
  ros::NodeHandle nh("~");
  RmuaPositionCommandBridge bridge(nh);
  ros::spin();
  return 0;
}

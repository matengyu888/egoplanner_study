#include <ros/ros.h>

#include <airsim_ros/VelCmd.h>

class RmuaVelTopicBridge
{
public:
  explicit RmuaVelTopicBridge(ros::NodeHandle& nh)
  {
    sub_ = nh.subscribe("vel_cmd_in", 50, &RmuaVelTopicBridge::callback, this);
    pub_ = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 20);
  }

private:
  void callback(const airsim_ros::VelCmdConstPtr& msg)
  {
    pub_.publish(*msg);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmua_vel_topic_bridge");
  ros::NodeHandle nh("~");
  RmuaVelTopicBridge bridge(nh);
  ros::spin();
  return 0;
}

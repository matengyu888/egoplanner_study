#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "send_fixed_goal_node");
    ros::NodeHandle nh("~"); // 私有节点句柄，读取本节点的参数（launch中设置的）

    // 2. 从参数服务器读取launch文件的配置参数（带默认值，防止参数未设置）
    double goal_x, goal_y, goal_z;
    std::string frame_id;
    bool is_cycle;
    double cycle_hz;

    // 读取目标点坐标（默认值与launch一致，可覆盖）
    nh.param<double>("goal_x", goal_x, 2.0);
    nh.param<double>("goal_y", goal_y, 3.0);
    nh.param<double>("goal_z", goal_z, 1.5);
    // 读取坐标系（默认map）
    nh.param<std::string>("frame_id", frame_id, "map");
    // 读取是否循环发布+循环频率
    nh.param<bool>("is_cycle", is_cycle, false);
    nh.param<double>("cycle_hz", cycle_hz, 0.2);

    // 3. 创建发布者：发布/move_base_simple/goal话题，队列大小10
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 10
    );

    // 4. 等待1秒，确保发布者与waypoint_generator建立连接（避免消息丢失）
    ros::Duration(1.0).sleep();

    // 5. 构造PoseStamped消息（核心结构不变）
    geometry_msgs::PoseStamped goal_msg;

    // 循环发布/一次性发布 逻辑分离
    if (is_cycle) {
        // 方案A：循环发布（按launch设置的频率）
        ros::Rate rate(cycle_hz);
        ROS_INFO("✅ 开始循环发布目标点，频率：%.2fHz", cycle_hz);
        while (ros::ok()) {
            // 每次发布更新时间戳（必须，否则规划器可能判定消息过期）
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = frame_id;
            // 设置从参数读取的位置
            goal_msg.pose.position.x = goal_x;
            goal_msg.pose.position.y = goal_y;
            goal_msg.pose.position.z = goal_z;
            // 姿态：水平无旋转（单位四元数）
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;

            // 发布消息
            goal_pub.publish(goal_msg);
            ROS_INFO("已发布：x=%.2f, y=%.2f, z=%.2f | 坐标系：%s",
                     goal_x, goal_y, goal_z, frame_id.c_str());

            rate.sleep(); // 按频率休眠
            ros::spinOnce();
        }
    } else {
        // 方案B：一次性发布（默认，推荐无人机导航）
        goal_msg.header.frame_id = frame_id;
        goal_msg.header.stamp = ros::Time::now();
        // 设置从参数读取的位置
        goal_msg.pose.position.x = goal_x;
        goal_msg.pose.position.y = goal_y;
        goal_msg.pose.position.z = goal_z;
        // 姿态：水平无旋转
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;
        goal_msg.pose.orientation.w = 1.0;

        // 发布消息
        if (ros::ok()) {
            goal_pub.publish(goal_msg);
            ROS_INFO("✅ 一次性发布固定目标点成功！");
            ROS_INFO("位置：x=%.2f, y=%.2f, z=%.2f（米）", goal_x, goal_y, goal_z);
            ROS_INFO("坐标系：%s | 姿态：水平朝前（0,0,0,1）", frame_id.c_str());
        }
        ros::spinOnce();
    }

    return 0;
}

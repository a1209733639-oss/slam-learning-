#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 1. 获取位置：odom坐标系下子帧的 x,y
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // 2. 从四元数 orientation 提取 yaw 航向角（单位：弧度）
    double yaw = tf::getYaw(msg->pose.pose.orientation);

    // 3. 线速度：x方向前进速度 m/s
    double vx = msg->twist.twist.linear.x;

    // 4. 角速度：绕Z轴旋转角速度 rad/s
    double wz = msg->twist.twist.angular.z;

    // 终端打印输出
    ROS_INFO(
        "x: %.3f m | y: %.3f m | yaw: %.3f rad | vx: %.3f m/s | wz: %.3f rad/s",
        x, y, yaw, vx, wz
    );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_monitor"); // 初始化ROS，节点名称 odom_monitor
    ros::NodeHandle nh;                    // 创建节点句柄

    // 订阅话题 /odom，队列大小10，回调函数 odomCallback
    ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);

    ROS_INFO("Waiting for /odom ...");

    ros::spin();   // 循环等待消息，阻塞；有消息到来自动执行回调

    return 0;
}


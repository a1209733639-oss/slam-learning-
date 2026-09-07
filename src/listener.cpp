#include <ros/ros.h>
#include <std_msgs/String.h>

// 回调函数：收到话题数据就自动执行这个函数
void callback(const std_msgs::String::ConstPtr& msg)
{
    // 打印接收到的字符串消息
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // 初始化ROS，节点名称 slam_listener
    ros::init(argc, argv, "slam_listener");
    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建订阅者
    // 订阅话题：/slam_message
    // 队列大小：10
    // 收到消息后调用 callback 函数
    ros::Subscriber sub = nh.subscribe("/slam_message", 10, callback);

    // ros::spin() ！！订阅者核心
    // 程序停在这里，不断监听话题，等待回调触发，Ctrl+C才退出
    ros::spin();
    return 0;
}

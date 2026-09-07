#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    // 1.初始化ROS，节点名叫 slam_talker
    ros::init(argc, argv, "slam_talker");
    
    // 2.创建节点句柄，相当于和ROS系统沟通的手柄
    ros::NodeHandle nh;

    // 3.创建发布者
    // 话题名称：/slam_message
    // 消息类型：std_msgs::String 字符串消息
    // 10：消息队列大小，缓存最多10条来不及发送的数据
    ros::Publisher pub = nh.advertise<std_msgs::String>("/slam_message", 10);

    // 4.设定循环频率 1Hz → 1秒发布1次
    ros::Rate rate(1);

    // 5.循环：ros::ok() 收到Ctrl+C就退出循环
    while (ros::ok())
    {
        // 定义消息对象
        std_msgs::String msg;
        // 给消息赋值
        msg.data = "Hello SLAM";
        // 发布话题
        pub.publish(msg);
        // ROS日志打印输出
        ROS_INFO("Publish: %s", msg.data.c_str());

        // 处理一次回调事件（发布者一般可写可不写，好习惯）
        ros::spinOnce();
        // 休眠，控制1Hz频率
        rate.sleep();
    }
    return 0;
}

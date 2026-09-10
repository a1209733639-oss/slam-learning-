#include <ros/ros.h>                          // ROS核心头文件
#include <sensor_msgs/PointCloud2.h>          // ROS点云消息格式
#include <tf/transform_listener.h>            // TF监听器，用来查询坐标变换
#include <pcl_ros/transforms.h>                // pcl_ros工具：直接对点云做坐标变换

// 点云转换节点类
class CloudTransformer
{
public:
    CloudTransformer()
    {
        // 订阅原始雷达点云话题 /points_raw，队列大小1，回调函数cloudCallback
        sub_ = nh_.subscribe(
            "/points_raw",
            1,
            &CloudTransformer::cloudCallback,
            this);

        // 发布变换完成后的点云，话题 /points_base，队列大小1
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/points_base",
            1);
    }

    // 收到点云消息触发的回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        sensor_msgs::PointCloud2 output;   // 存储变换之后输出的点云

        try
        {
            /**
             * pcl_ros::transformPointCloud(目标帧,输入点云,输出点云,tf监听器)
             * 第1参数：目标坐标系 base_footprint
             * 第2参数：*cloud 原始输入点云（frame_id一般是velodyne）
             * 第3参数：output输出变换后的点云
             * 第4参数：TF监听器listener，内部自动查询 velodyne → base_footprint 的TF变换
             */
            pcl_ros::transformPointCloud(
                "base_footprint",
                *cloud,
                output,
                listener_);

            // 手动设置输出点云坐标系标识
            output.header.frame_id = "base_footprint";

            // 发布转换完成的点云
            pub_.publish(output);
        }
        catch(tf::TransformException &ex)
        {
            // TF查询失败捕获异常：没有变换、时间外插、TF延迟等，打印警告不崩溃
            ROS_WARN("%s",ex.what());
        }
    }

private:
    ros::NodeHandle nh_;                // 节点句柄
    ros::Subscriber sub_;                // 点云订阅器
    ros::Publisher pub_;                // 点云发布器
    tf::TransformListener listener_;     // TF变换监听器，后台缓存所有TF树
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"cloud_transformer"); // 初始化ROS，节点名 cloud_transformer
    CloudTransformer node;                    // 实例化类，自动执行构造函数，完成订阅发布
    ros::spin();                              // 循环等待回调，阻塞运行
    return 0;
}


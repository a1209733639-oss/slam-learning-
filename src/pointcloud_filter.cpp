

#include <ros/ros.h>                 // ROS基础
#include <sensor_msgs/PointCloud2.h> // ROS点云消息
#include <pcl/point_cloud.h>         // PCL点云容器
#include <pcl/point_types.h>         // PCL点类型：PointXYZI（带强度）
#include <pcl/filters/voxel_grid.h>  // PCL体素滤波算法
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg / toROSMsg 转换函数


class PointCloudFilter
{
// 用类封装节点，把订阅、发布、回调函数封装在一起，是 ROS 工程常用写法
public:
    PointCloudFilter()
    {
        // 订阅话题 /points_raw，队列大小1，回调函数cloudCallback
        cloud_sub_ = nh_.subscribe(
            "/points_raw",
            1,
            &PointCloudFilter::cloudCallback,
            this
        );
        // 发布滤波之后点云话题 /points_filtered，队列1
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/points_filtered",
            1
        );
        ROS_INFO("PointCloud Filter Node Started.");
    }
private:
    ros::NodeHandle nh_;        // ROS句柄
    ros::Subscriber cloud_sub_;// 订阅者
    ros::Publisher cloud_pub_;  // 发布者

void cloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
    
        // 定义两个PCL智能指针：原始点云、滤波后点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );
        // cl::PointXYZI：点类型，包含 x,y,z,intensity(强度)`
        
        // ROS PointCloud2 -> PCL
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZI> voxel;

        voxel.setInputCloud(cloud);
        // setLeafSize(0.1,0.1,0.1)：体素格子大小 **10cm ×10cm ×10cm**
        // filter()执行滤波，结果存入filtered_cloud
        voxel.setLeafSize(
            0.10f,
            0.10f,
            0.10f
        );

        voxel.filter(*filtered_cloud);

        ROS_INFO(
            "Raw: %lu points -> Filtered: %lu points",
            cloud->size(),
            filtered_cloud->size()
        );

        // PCL -> ROS PointCloud2
        sensor_msgs::PointCloud2 output_msg;

        pcl::toROSMsg(
            *filtered_cloud,
            output_msg
        );

        // 保留原始点云坐标系和时间
        output_msg.header = cloud_msg->header;

        cloud_pub_.publish(output_msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_filter"); // 初始化节点，节点名pointcloud_filter
    PointCloudFilter node;                       // 创建类实例，构造函数执行，订阅发布建立
    ros::spin();                                 // 循环等待回调，阻塞
    return 0;
}


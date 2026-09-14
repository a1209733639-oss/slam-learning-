#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

ros::Publisher pub;  //全局发布器，回调里使用

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //ROS消息转PCL点云，ROS网络消息不能直接给PCL算法用
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    //体素滤波格子大小，单位米，降采样减少点云数量
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*filtered);

    sensor_msgs::PointCloud2 output;
    //PCL处理完转回ROS消息，用于发布
    pcl::toROSMsg(*filtered, output);

    //复制坐标系、时间戳，丢了TF/SLAM直接异常
    output.header = msg->header;

    pub.publish(output);
    ROS_INFO("raw:%ld filtered:%ld", cloud->size(), filtered->size());
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "cloud_filter_node");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/points_filtered",1);
    //订阅原始点云话题，收到数据自动执行cloudCallback
    ros::Subscriber sub = nh.subscribe("/points_raw", 1, cloudCallback);

    //消息循环，驱动回调函数执行，没有spin回调不会跑
    ros::spin();
    return 0;
}


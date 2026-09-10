#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>       // TF监听，获取坐标变换
#include <pcl_ros/transforms.h>           // pcl_ros::transformPointCloud 点云TF变换函数
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg / toROSMsg

class CloudTransformFilter
{
public:
CloudTransformFilter()
{
sub_=nh_.subscribe("/points_raw",1,&CloudTransformFilter::callback,this);
pub_=nh_.advertise<sensor_msgs::PointCloud2>("/points_filtered",1);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    sensor_msgs::PointCloud2 cloud_base;
    try
    {
        // =========步骤1：点云坐标变换 velodyne → base_footprint =========
        pcl_ros::transformPointCloud(
            "base_footprint",   // 目标坐标系
            *cloud,              // 输入原始点云(velodyne_link)
            cloud_base,          // 输出变换之后的ROS点云消息
            listener_            // TF监听器，用来查询TF变换矩阵
        );
       // =========步骤2 ROS PointCloud2 → PCL点云 =========
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(cloud_base, *pcl_cloud);
       // =========步骤3 VoxelGrid体素滤波下采样 =========
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(pcl_cloud);
        voxel.setLeafSize(0.1,0.1,0.1); //体素格子10cm
        voxel.filter(*filtered);

       // =========步骤4 PCL转回ROS PointCloud2消息 =========
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered, output);

        output.header.frame_id="base_footprint";
        output.header.stamp = cloud_base.header.stamp;
        pub_.publish(output);
        ROS_INFO("raw:%ld filtered:%ld",pcl_cloud->size(),filtered->size());
    }
    catch(tf::TransformException &ex) //捕获TF查询失败异常
    {
        ROS_WARN("%s",ex.what());
    }
}

private:
ros::NodeHandle nh_;
ros::Subscriber sub_;
ros::Publisher pub_;
tf::TransformListener listener_; //TF监听器，持续接收系统TF数据

};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"cloud_transform_filter");
    CloudTransformFilter node;
    ros::spin();
    return 0;
}



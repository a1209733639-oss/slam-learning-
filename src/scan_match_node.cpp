#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>

class NDTMatcher
{
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
    // 累计全局位姿：odom坐标系下的变换矩阵
    Eigen::Matrix4f pose;

public:
    NDTMatcher()
    {
        // 订阅滤波后点云
        cloud_sub = nh.subscribe("/points_filtered", 1, &NDTMatcher::cloudCallback, this);
        // 发布NDT计算出来的里程计话题
        odom_pub = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 10);
        // 初始位姿设为单位矩阵，原点出发
        pose = Eigen::Matrix4f::Identity();
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        // 第一帧，没有历史点云，只保存直接返回
        if(previous_cloud == nullptr)
        {
            previous_cloud = current_cloud;
            ROS_INFO("Initialize first cloud");
            return;
        }

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setInputSource(previous_cloud);   // 源：上一帧点云
        ndt.setInputTarget(current_cloud);    // 目标：当前帧点云

        ndt.setResolution(1.0f);              // NDT网格分辨率 m
        ndt.setMaximumIterations(30);         // 最大迭代次数

        pcl::PointCloud<pcl::PointXYZ> aligned;
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        // align第二个参数：配准初始猜测变换
        ndt.align(aligned, guess);

        if(ndt.hasConverged())
        {
            // delta：上一帧到当前帧相对变换
            Eigen::Matrix4f delta = ndt.getFinalTransformation();
            //位姿累积，矩阵乘法更新全局pose
            pose = pose * delta;

            // 发布odom消息 + 广播TF odom->base_link
            publishOdom(msg->header.stamp);

            // 从4×4矩阵提取yaw航向角
            Eigen::Matrix3f R = pose.block<3,3>(0,0);
            float yaw = atan2(R(1,0), R(0,0));
            ROS_INFO("Pose x %.3f y %.3f yaw %.3f", pose(0,3), pose(1,3), yaw);
        }
        else
        {
            ROS_WARN("NDT match failed, skip this frame");
        }
        // 更新历史点云
        previous_cloud = current_cloud;
    }

    void publishOdom(ros::Time stamp)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 填入平移
        odom.pose.pose.position.x = pose(0,3);
        odom.pose.pose.position.y = pose(1,3);
        odom.pose.pose.position.z = pose(2,3);

        
        Eigen::Matrix3f rot_mat = pose.block<3,3>(0,0);
        tf::Matrix3x3 tf_rot(
            rot_mat(0,0), rot_mat(0,1), rot_mat(0,2),
            rot_mat(1,0), rot_mat(1,1), rot_mat(1,2),
            rot_mat(2,0), rot_mat(2,1), rot_mat(2,2)
        );
        tf::Quaternion q;
        tf_rot.getRotation(q);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub.publish(odom);

        // 广播TF odom → base_link
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
        transform.setRotation(q);
        tf_pub.sendTransform(tf::StampedTransform(transform, stamp, "odom", "base_link"));
    }
};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "scan_match_node");
    NDTMatcher matcher;
    ros::spin();
    return 0;
}


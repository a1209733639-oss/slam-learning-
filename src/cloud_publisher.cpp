#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>


int main(int argc,char** argv)
{

    ros::init(
        argc,
        argv,
        "cloud_publisher"
    );


    ros::NodeHandle nh;


    ros::Publisher pub =
        nh.advertise<sensor_msgs::PointCloud2>(
            "/points_raw",
            1
        );


    ros::Rate rate(5);


    float offset = 0;


    while(ros::ok())
    {

        pcl::PointCloud<pcl::PointXYZ> cloud;


        // 模拟一片墙面点云

        for(float x=0;x<5;x+=0.1)
        {

            for(float y=0;y<5;y+=0.1)
            {

                cloud.push_back(
                    pcl::PointXYZ(
                        x+offset,
                        y,
                        0
                    )
                );

            }

        }


        sensor_msgs::PointCloud2 msg;


        pcl::toROSMsg(
            cloud,
            msg
        );


        msg.header.stamp =
            ros::Time::now();


        msg.header.frame_id =
            "velodyne";


        pub.publish(msg);



        // 模拟机器人移动

        offset += 0.05;


        rate.sleep();

    }


    return 0;

}

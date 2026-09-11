#include <iostream>
#include <pcl/point_types.h>        // PCL点类型定义，PointXYZ
#include <pcl/point_cloud.h>        // 点云容器 PointCloud
#include <pcl/registration/icp.h>   // ICP配准算法头文件

int main()
{
    // 创建源点云智能指针：待变换的点云（待对齐）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    // 创建目标点云智能指针：参考点云，source要对齐到target坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    // 向源点云插入3个简单2D平面测试点 z=0
    cloud_source->push_back(
        pcl::PointXYZ(0,0,0)
    );
    cloud_source->push_back(
        pcl::PointXYZ(1,0,0)
    );
    cloud_source->push_back(
        pcl::PointXYZ(0,1,0)
    );

    // 目标点云：人为给source施加平移(+0.5, +0.2, 0)得到target
    // ICP任务：从source、target反解出这个变换矩阵
    cloud_target->push_back(
        pcl::PointXYZ(0.5,0.2,0)
    );
    cloud_target->push_back(
        pcl::PointXYZ(1.5,0.2,0)
    );
    cloud_target->push_back(
        pcl::PointXYZ(0.5,1.2,0)
    );

    // 实例化ICP对象
    // 模板参数：<源点类型，目标点类型>，这里都是PointXYZ
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;

    // 设置输入源点云：需要被配准、被变换的点云
    icp.setInputSource(cloud_source);
    // 设置目标点云：参考基准点云
    icp.setInputTarget(cloud_target);

    // aligned：存储配准完成后的输出点云，source经过求解变换后的点
    pcl::PointCloud<pcl::PointXYZ> aligned;

    // 执行ICP迭代配准，结果存入aligned
    // 内部循环：1.找最近点 2.计算变换矩阵 3.把source点云乘矩阵更新；迭代直到收敛/达到最大迭代次数
    icp.align(aligned);

    // hasConverged()：返回bool，true1 代表ICP迭代达到收敛条件；false0代表未收敛
    std::cout
    <<"ICP has converged:"
    <<icp.hasConverged()
    <<std::endl;

    // getFitnessScore() 适配分数（匹配误差）：所有对应点对的均方误差MSE
    // 数值越小代表source变换后和target重合度越好；完全重合接近0
    std::cout
    <<"Fitness score:"
    <<icp.getFitnessScore()
    <<std::endl;

    // getFinalTransformation() 获取4×4齐次变换矩阵T
    // source经过 T * source_point 得到对齐到target坐标系下的点
    // 矩阵形式：[R t; 0 0 0 1]，R旋转，t平移
    std::cout
    <<"Transformation:"
    <<std::endl;
    std::cout
    <<icp.getFinalTransformation()
    <<std::endl;

    return 0;
}


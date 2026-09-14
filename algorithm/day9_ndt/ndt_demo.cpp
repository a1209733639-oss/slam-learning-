#include <iostream>
#include <pcl/point_types.h>        // PCL点类型，定义pcl::PointXYZ
#include <pcl/point_cloud.h>        // 点云容器模板类 PointCloud
#include <pcl/registration/ndt.h>   // NDT正态分布变换配准算法头文件

int main()
{
    // source：源点云，待配准、待变换的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(
        new pcl::PointCloud<pcl::PointXYZ>
    );
    // target：目标点云，参考基准；source要对齐到target坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    // 给源点云填生成一个面的点云
for(float x=0;x<5;x+=0.2)
{
    for(float y=0;y<5;y+=0.2)
    {
        source->push_back(
            pcl::PointXYZ(x,y,0)
        );
    }
}

    // 目标点云：源点云整体平移，5×5米
    // NDT任务：根据source和target，求解这个4×4变换矩阵
for(float x=0;x<5;x+=0.2)
{
    for(float y=0;y<5;y+=0.2)
    {
        target->push_back(
            pcl::PointXYZ(
                x+0.5,
                y+0.2,
                0
            )
        );
    }
}
    // 实例化NDT配准对象 NormalDistributionsTransform
    // 模板参数：<源点类型，目标点类型>，此处都是PointXYZ
    // NDT原理：把target点云划分体素网格，每个网格拟合高斯正态分布；迭代优化位姿让source点落在target高斯分布概率最大
    pcl::NormalDistributionsTransform<
        pcl::PointXYZ,
        pcl::PointXYZ
    > ndt;

    ndt.setInputSource(source);     // 设置待变换源点云
    ndt.setInputTarget(target);     // 设置参考目标点云

    // 设置NDT体素网格分辨率，单位m；每个格子大小0.5米
    // 关键参数：网格太大拟合粗糙；网格太小，内存大、对噪声敏感
    ndt.setResolution(0.5);
    ndt.setMaximumIterations(100);

    // output：保存配准完成之后，经过变换的source点云结果
    pcl::PointCloud<pcl::PointXYZ> output;

    // 执行NDT迭代配准，结果写入output
    // 可传入第二个参数作为初始位姿guess；不给则默认单位矩阵作为初始值
    ndt.align(output);

    // hasConverged() 返回true/false，代表算法迭代是否达到收敛条件
    std::cout
    <<"NDT converged:"
    <<ndt.hasConverged()
    <<std::endl;

    // getFitnessScore() 适配得分：变换后源点与目标之间的平均误差，数值越小重合越好
    std::cout
    <<"Score:"
    <<ndt.getFitnessScore()
    <<std::endl;

    // getFinalTransformation() 获取4×4齐次变换矩阵 T
    // 公式：P_source_transformed = T * P_source
    // 矩阵结构：[R  t; 0 0 0 1]  R旋转矩阵，t平移向量
    std::cout
    <<"Transformation:"
    <<std::endl;
    std::cout
    <<ndt.getFinalTransformation()
    <<std::endl;

    return 0;
}


#include <iostream>

#include <pcl/point_cloud.h>      //PCL 点云容器头文件，pcl::PointCloud类定义
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    //push_back()向点云中插入点   插入顺序对应索引：0,1,2,3
    
    cloud->push_back(
        pcl::PointXYZ(1,1,1)
    );

    cloud->push_back(
        pcl::PointXYZ(2,2,2)
    );

    cloud->push_back(
        pcl::PointXYZ(5,5,5)
    );

    cloud->push_back(
        pcl::PointXYZ(8,8,8)
    );

    //实例化 KD‑Tree 对象；模板参数<pcl::PointXYZ>告诉树处理什么类型的点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  

    kdtree.setInputCloud(cloud);  //构建 KD‑Tree 索引！即分割区域

    pcl::PointXYZ searchPoint;  //定义查询点

    searchPoint.x = 2.1;
    searchPoint.y = 2.1;
    searchPoint.z = 2.1;


    int K = 1;  //K 近邻参数，取最近1个点


    std::vector<int> index(K);

    std::vector<float> distance(K);  //两个输出容器：index：保存搜索得到的点在原始点云里的索引编号     distance：保存距离的平方值



    if(kdtree.nearestKSearch(
        searchPoint,
        K,
        index,
        distance
    )>0)  //K 近邻搜索函数   输入：查询点、K ；输出：填入 index、distance
    {

        std::cout
        <<"Nearest index:"
        <<index[0]
        <<std::endl;


        std::cout
        <<"Distance:"
        <<distance[0]
        <<std::endl;

    }


    return 0;
}

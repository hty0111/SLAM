/**
 * @Description: 写入pcd文件
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-17 13:56:45
 */


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;    // 数量
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (auto & point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("../resource/test_pcd.pcd", cloud);
    std::cout << "Saved " << cloud.size() << " data points." << std::endl;
    for (const auto & point : cloud)
        std::cout << "\t" << point.x << "\t" << point.y << "\t" << point.z << std::endl;

    return 0;
}

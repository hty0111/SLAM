/**
 * @Description: 变换矩阵
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-17 14:36:45
 */

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile("../resource/cube.ply", *source_cloud) < 0)
    {
        std::cout << "Path error!\n";
        return -1;
    }

    // define a transformation
    Eigen::Affine3f transform= Eigen::Affine3f::Identity();
    transform.translation() << 2.5, 0.0, 0.0;
    transform.rotate(Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()));
    std::cout << "transform using Affine3f:\n" << transform.matrix() << std::endl;

    // execute the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, * transformed_cloud, transform);

    // visualize
    pcl::visualization::PCLVisualizer viewer("Matrix transformation");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    while (!viewer.wasStopped())
        viewer.spinOnce();

    return 0;
}
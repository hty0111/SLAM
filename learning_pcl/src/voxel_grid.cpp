/**
 * Description: 滤波器
 * version: v1.0
 * Author: HTY
 * Date: 2022-12-09 19:40:24 
 */

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    // load PCD file
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile("../resource/table_scene_lms400.pcd", *cloud);

    // voxel grid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01, 0.01, 0.01);
    filter.filter(*cloud_filtered);

    pcl::PCDWriter writer;
    writer.write("../resource/table_scene_lms400_voxel_downsampled.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_xyz);

    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud_xyz, "voxel grid", false);
    while (!viewer.wasStopped())
        viewer.spinOnce();

    return 0;
}
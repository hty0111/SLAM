/**
 * Description:
 * version: v1.0
 * Author: HTY
 * Date: 2022-12-09 20:52:58 
 */

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    // load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("../resource/table_scene_lms400.pcd", *cloud);

    // statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(50);
    filter.setStddevMulThresh(1.0);
    filter.filter(*cloud_filtered);

    pcl::PCDWriter writer;
    writer.write("../resource/table_scene_lms400_statistical_outlier_removal.pcd", *cloud_filtered, false);

    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud_filtered, "outlier removal", false);
    while (!viewer.wasStopped())
        viewer.spinOnce();

    return 0;
}
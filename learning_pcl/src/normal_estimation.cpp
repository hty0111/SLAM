/**
 * @Description: 法线估计
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-20 22:39:21
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../resource/table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped())
        viewer.spinOnce();

    return 0;
}


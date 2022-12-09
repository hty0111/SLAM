/**
 * @Description: 法线估计
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-20 22:39:21
 */

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../resource/table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ine;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ine.setSearchMethod(tree);
    ine.setNormalEstimationMethod(ine.AVERAGE_3D_GRADIENT);
    ine.setMaxDepthChangeFactor(0.02f);
    ine.setNormalSmoothingSize(10.0f);
//    ine.setRadiusSearch(0.05);
    ine.setInputCloud(cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ine.compute(*normals);

    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped())
        viewer.spinOnce();

    return 0;
}


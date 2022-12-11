/**
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-12-10 16:45:11
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../resource/bunny.pcd", *cloud);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    mls.setSearchMethod(tree);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchRadius(0.03);
    mls.setComputeNormals(true);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointNormal>);
    mls.process(*cloud_mls);
    pcl::io::savePCDFile ("../resource/bunny_mls.pcd", *cloud_mls);

    pcl::visualization::PCLVisualizer viewer("Cluster");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal> (cloud, cloud_mls, 25, 0.15, "normals");
    viewer.spin();

    return 0;
}


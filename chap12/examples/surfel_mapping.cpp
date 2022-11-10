/*
 * @Description: surfel地图
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-08 20:58:29
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/mls.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;


SurfelCloudPtr reconstructSurface(const PointCloudPtr & input_cloud, float radius, int polynomial_order)
{
    pcl::MovingLeastSquares<PointT, SurfelT> mls;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(radius * radius);
    mls.setPolynomialOrder(polynomial_order);
    mls.setInputCloud(input_cloud);
    SurfelCloudPtr output_cloud(new SurfelCloud);
    mls.process(*output_cloud);
    return output_cloud;
}


pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr & surfels)
{
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);

    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    gp3.setSearchRadius(0.05);  // maximum edge length
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);   // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);     // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);  // 120 degrees
    gp3.setNormalConsistency(true);

    // get result
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);
    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);
    return triangles;
}


int main()
{
    PointCloudPtr point_cloud(new PointCloud);
    pcl::io::loadPCDFile("../map.pcd", *point_cloud);
    std::cout << "loaded points number: " << point_cloud->size() << std::endl;

    // Compute surface elements
    float mls_radius = 0.05;
    int polynomial_order = 2;
    auto surfels = reconstructSurface(point_cloud, mls_radius, polynomial_order);

    // Compute a greedy surface triangulation
    pcl::PolygonMeshPtr mesh = triangulateMesh(surfels);

    // display
    pcl::visualization::PCLVisualizer vis;
    vis.addPolylineFromPolygonMesh(*mesh, "mesh frame");
    vis.addPolygonMesh(*mesh, "mesh");
    vis.resetCamera();
    vis.spin();

    return 0;
}
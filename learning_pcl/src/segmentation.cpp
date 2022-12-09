/**
 * Description:
 * version: v1.0
 * Author: HTY
 * Date: 2022-12-09 22:36:32 
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    // Read data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("../resource/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cout << "cloud size: " << cloud->size() << std::endl;

    // Build a passthrough filter to remove spurious NaNs and scene background
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);
    std::cout << "cloud size after passthrough filtered: " << cloud_filtered->size() << std::endl;

    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_plane(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals_plane);

    // Create the segmentation object for the planar model
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals_plane);
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cout << "plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cylinder(new pcl::PointCloud<pcl::Normal>);
    extract.setNegative(true);
    extract.filter(*cloud_filtered_cylinder);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals_plane);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals_cylinder);

    // Create the segmentation object for cylinder segmentation
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud(cloud_filtered_cylinder);
    seg.setInputNormals(cloud_normals_cylinder);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Extract the cylinder inliers from the input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_filtered_cylinder);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    pcl::PCDWriter writer;
    writer.write("../resource/table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
    writer.write("../resource/table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);

    pcl::visualization::PCLVisualizer viewer;
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filtered_cylinder, cloud_normals_cylinder);
    viewer.addPointCloud(cloud_cylinder);
    viewer.addCoordinateSystem();
    viewer.spin();


    return 0;
}
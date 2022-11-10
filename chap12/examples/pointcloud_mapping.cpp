/*
 * @Description: 点云地图
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-08 15:55:03
 */


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <iostream>

int main()
{
    std::vector<cv::Mat> color_images, depth_images;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;  // 必须要用allocator做转换

    std::ifstream fin("../data/pose.txt");
    for (int i = 0; i < 5; ++i)
    {
        boost::format fmt("../data/%s/%d.png");
        color_images.push_back(cv::imread((fmt % "color" % (i + 1)).str()));
        depth_images.push_back(cv::imread((fmt % "depth" % (i + 1)).str(), cv::IMREAD_UNCHANGED));

        double data[7] = {0};
        for (double & d : data)
            fin >> d;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 相机内参
    double fx = 481.2, fy = -480.0, cx = 319.5, cy = 239.5, depth_scale = 5000.0;

    // 定义点云使用格式
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;

    PointCloudType::Ptr point_cloud(new PointCloudType);
    for (int i = 0; i < 5; ++i)
    {
        std::cout << "converting image " << i + 1 << std::endl;

        PointCloudType::Ptr current(new PointCloudType);
        cv::Mat color = color_images[i];
        cv::Mat depth = depth_images[i];
        Eigen::Isometry3d T_c2w = poses[i];

        for (int v = 0; v < color.rows; ++v)
            for (int u = 0; u < color.cols; ++u)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) 
                    continue;   // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depth_scale; // 太坑爹了，必须要先把d转成double
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d point_w = T_c2w * point;

                PointType p;
                p.x = point_w[0];
                p.y = point_w[1];
                p.z = point_w[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistic removal
        PointCloudType::Ptr temp(new PointCloudType);
        pcl::StatisticalOutlierRemoval<PointType> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*temp);    // 滤波器的结果输出给temp
        (*point_cloud) += *temp;    // 点云相加即为拼接
    }
    point_cloud->is_dense = false;
    std::cout << "共有点云：" << point_cloud->size() << std::endl;

    // voxel filter
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(0.03, 0.03, 0.03); // resolution
    PointCloudType::Ptr temp(new PointCloudType);
    voxel_filter.setInputCloud(point_cloud);
    voxel_filter.filter(*temp);
    temp->swap(*point_cloud);   // 将滤波后的结果temp与原始数据交换

    std::cout << "体素滤波后，共有点云：" << point_cloud->size() << std::endl;
    pcl::io::savePCDFileBinary("../map.pcd", *point_cloud);

    return 0;
}

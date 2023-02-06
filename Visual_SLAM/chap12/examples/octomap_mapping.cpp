/*
 * @Description: 八叉树地图
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-08 22:20:17
 */


#include <octomap/octomap.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

    octomap::OcTree tree(0.01); // resolution
    for (int i = 0; i < 5; ++i)
    {
        std::cout << "converting image " << i + 1 << std::endl;

        cv::Mat color = color_images[i];
        cv::Mat depth = depth_images[i];
        Eigen::Isometry3d T_c2w = poses[i];
        octomap::Pointcloud cloud;

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
                cloud.push_back(point_w[0], point_w[1], point_w[2]);
            }

        // 点云存入八叉树，给定原点计算投射线
        tree.insertPointCloud(cloud, octomap::point3d(T_c2w(0, 3), T_c2w(1, 3), T_c2w(2, 3)));
    }

    tree.updateInnerOccupancy();
    tree.writeBinary("../octomap.bt");

    return 0;
}
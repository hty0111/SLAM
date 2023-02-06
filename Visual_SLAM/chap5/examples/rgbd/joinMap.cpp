/*
 * @Description: rgbd相机（第一版book）
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-25 10:58:35
 */

#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define CX 325.5
#define CY 253.5
#define FX 518.0
#define FY 519.0
#define DEPTH 1000.0

int main()
{
    vector<cv::Mat> colorImgs, depthImgs;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    // 获取位姿
    ifstream fin("../pose.txt");
    if (!fin)
    {
        cout << "Path error." << endl;
        return -1;
    }

    for (int i = 0; i < 5; i++)
    {
        boost::format fmt("../%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt%"color"%(i+1)%"png").str()));
        depthImgs.push_back(cv::imread((fmt%"depth"%(i+1)%"pgm").str(), -1));

        double data[7] = {0};
        for (auto& d: data)     // 一定要用引用 &
            fin >> d;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算点云并拼接
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < 5; i++)
    {
        cout << "转换图像：" << i+1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short> (v)[u];  // 深度
                if (d == 0) continue;
                Eigen::Vector3d point;
                point[2] = double(d) / DEPTH;   // z = d
                point[0] = (u - CX) * point[2] / FX;    // x = (u-cx)/fx*z
                point[1] = (v - CY) * point[2] / FY;    // y = (u-cy)/fy*z
                Eigen::Vector3d pointWorld = T * point;

                PointT p;
                p.x = (float)pointWorld[0];
                p.y = (float)pointWorld[1];
                p.z = (float)pointWorld[2];
                p.b = color.data[v*color.step + u*color.channels()];
                p.g = color.data[v*color.step + u*color.channels()+1];
                p.r = color.data[v*color.step + u*color.channels()+2];
                pointCloud->points.push_back(p);
            }
    }
    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点" << endl;
    pcl::io::savePCDFileBinary("../map.pcd", *pointCloud);

    return 0;

}

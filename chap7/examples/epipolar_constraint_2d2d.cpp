/*
 * @Description: 对极约束 2D-2D
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-27 17:23:43
 */

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

void find_feature_matches(
        const cv::Mat & img1,
        const cv::Mat & img2,
        std::vector<cv::KeyPoint> & key_points1,
        std::vector<cv::KeyPoint> & key_points2,
        std::vector<cv::DMatch> & matches);


void pose_estimation2d2d(
        std::vector<cv::KeyPoint> & key_points1,
        std::vector<cv::KeyPoint> & key_points2,
        std::vector<cv::DMatch> & matches,
        const cv::Mat & R,
        const cv::Mat & t,
        const cv::Mat & K);


/*!
 * 像素坐标转相机归一化坐标
 * @param p 像素坐标
 * @param K 相机内参
 * @return 相机归一化坐标
 */
cv::Point2d pixel2camera(const cv::Point2d & p, const cv::Mat & K);


void triangulation(
        const std::vector<cv::KeyPoint> & key_points1,
        const std::vector<cv::KeyPoint> & key_points2,
        const std::vector<cv::DMatch> & matches,
        const cv::Mat & R,
        const cv::Mat & t,
        const cv::Mat & K,
        std::vector<cv::Point3d> & points);


inline cv::Scalar get_color(double depth)
{
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return {255 * depth / th_range, 0, 255 * (1 - depth / th_range)};
}


int main()
{
    // 读取图像
    cv::Mat img1 = cv::imread("../1.png");
    cv::Mat img2 = cv::imread("../2.png");
    assert(img1.data && img2.data && "Path error.");

    // 特征匹配
    std::vector<cv::KeyPoint> key_points1, key_points2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img1, img2, key_points1, key_points2, matches);
    std::cout << "find " << matches.size() << " matches.\n";

    // 估计两张图像运动
    const cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);    // 相机内参
    cv::Mat_<double> R(3, 3), t(3, 1);
    pose_estimation2d2d(key_points1, key_points2, matches, R, t, K);

    // 三角化
    std::vector<cv::Point3d> points;
    triangulation(key_points1, key_points2, matches, R, t, K, points);

    // 验证 E = t^R * scale
    cv::Mat t_hat = (cv::Mat_<double>(3, 3) <<
            0, -t.at<double>(2, 0), t.at<double>(1, 0),
            t.at<double>(2, 0), 0, -t.at<double>(0, 0),
            -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    std::cout << "t^R = " << t_hat * R << std::endl;

//    Eigen::Vector3d t_eigen;
//    cv::cv2eigen(t, t_eigen);
//    auto t_eigen_hat = Sophus::SO3d::hat(t_eigen);
//    Eigen::Matrix<double, 3, 3> R_eigen;
//    cv::cv2eigen(R, R_eigen);
//    std::cout << "t^R = " << t_eigen_hat * R_eigen << std::endl;

    // 验证对极约束
    for (auto m : matches)
    {
        cv::Point2d pt1 = pixel2camera(key_points1[m.queryIdx].pt, K);
        cv::Mat P1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);   // P1 = [X1, Y1, 1]^{T}
        cv::Point2d pt2 = pixel2camera(key_points2[m.trainIdx].pt, K);
        cv::Mat P2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);   // P2 = [X2, Y2, 1]^{T}
        // P1^{T} K^{-T} t^ R K^{-1} P2
        cv::Mat equation_result = P2.t() * t_hat * R * P1;  // TODO 没懂
        std::cout << "epipolar constraint = " << equation_result << std::endl;
    }

    // 验证三角化点与特征点的重投影关系
    cv::Mat img_plot1 = img1.clone();
    cv::Mat img_plot2 = img2.clone();
    for (int i = 0; i < matches.size(); ++i)
    {
        // 图一
        double depth1 = points[i].z;
        cv::circle(img_plot1, key_points1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        // 图二
        cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        double depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img_plot2, key_points2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("image 1", img_plot1);
    cv::imshow("image 2", img_plot2);
    cv::waitKey(0);

    return 0;
}


void find_feature_matches(
        const cv::Mat & img1,
        const cv::Mat & img2,
        std::vector<cv::KeyPoint> & key_points1,
        std::vector<cv::KeyPoint> & key_points2,
        std::vector<cv::DMatch> & matches)
{
    // 检测 Oriented FAST 角点
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    detector->detect(img1, key_points1);
    detector->detect(img2, key_points2);

    // 根据角点位置计算 BRIEF 描述子
    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    descriptor->compute(img1, key_points1, descriptors1);
    descriptor->compute(img2, key_points2, descriptors2);

    // 使用 Hamming 距离对 BRIEF 描述子进行匹配
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> match;
    matcher->match(descriptors1, descriptors2, match);

    // 筛选匹配点
    auto min_max = minmax_element(match.begin(), match.end(),
                                  [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    for (int i = 0; i < descriptors1.rows; i++)
        if (match[i].distance <= std::max(2 * min_dist, 30.0))
            matches.push_back(match[i]);
}


void pose_estimation2d2d(
        std::vector<cv::KeyPoint> & key_points1,
        std::vector<cv::KeyPoint> & key_points2,
        std::vector<cv::DMatch> & matches,
        const cv::Mat & R,
        const cv::Mat & t,
        const cv::Mat & K)
{
    // 把匹配点类型从 KeyPoint 转换为 Point2f
    std::vector<cv::Point2f> points1, points2;
    for (auto & m : matches)
    {
        points1.push_back(key_points1[m.queryIdx].pt);
        points2.push_back(key_points2[m.trainIdx].pt);
    }

    // 计算基础矩阵 Fundamental Matrix
    cv::Mat fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    std::cout << "fundamental matrix = " << std::endl << fundamental_matrix << std::endl;

    // 计算本质矩阵 Essential Matrix
    double focal_length = K.at<double>(1, 1);   // f = fx = fy
    cv::Point2d principal_point(K.at<double>(0, 2), K.at<double>(1, 2));    // (cx, cy)
    cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    std::cout << "essential matrix = " << std::endl << essential_matrix << std::endl;

    // 计算单应矩阵 Homography Matrix
    cv::Mat homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
    std::cout << "homography matrix = " << std::endl << homography_matrix << std::endl;

    // 从本质矩阵中恢复 R & t
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    std::cout << "R = " << std::endl << R << std::endl;
    std::cout << "t = " << std::endl << t << std::endl;
}


cv::Point2d pixel2camera(const cv::Point2d & p, const cv::Mat & K)
{
    return {(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)};
}


void triangulation(
        const std::vector<cv::KeyPoint> & key_points1,
        const std::vector<cv::KeyPoint> & key_points2,
        const std::vector<cv::DMatch> & matches,
        const cv::Mat & R,
        const cv::Mat & t,
        const cv::Mat & K,
        std::vector<cv::Point3d> & points)
{
    cv::Mat T1 = (cv::Mat_<double>(3, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    std::vector<cv::Point2f> points1, points2;
    for (auto & m : matches)
    {
        points1.push_back(pixel2camera(key_points1[m.queryIdx].pt, K));
        points2.push_back(pixel2camera(key_points2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, points1, points2, pts_4d);

    // 转换为非齐次坐标
    for( int i = 0; i < pts_4d.cols; ++i)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
        points.push_back(p);
    }
}
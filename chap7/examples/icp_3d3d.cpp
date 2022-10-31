/*
 * @Description: ICP 3D-3D
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-30 14:45:38
 */

#include <Eigen/Core>
#include <Eigen/SVD>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <utility>


void find_feature_matches(
        const cv::Mat & img1,
        const cv::Mat & img2,
        std::vector<cv::KeyPoint> & key_points1,
        std::vector<cv::KeyPoint> & key_points2,
        std::vector<cv::DMatch> & matches);


cv::Point2d pixel2camera(const cv::Point2d & p, const cv::Mat & K);


void pose_estimation_3d3d(
        const std::vector<cv::Point3f> & pts1,
        const std::vector<cv::Point3f> & pts2,
        cv::Mat & R,
        cv::Mat & t
);


void bundleAdjustment(
        const std::vector<cv::Point3f> & pts1,
        const std::vector<cv::Point3f> & pts2,
        cv::Mat & R,
        cv::Mat & t
);


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

    // 建立3D点
    cv::Mat depth_img1 = cv::imread("../1_depth.png", cv::IMREAD_UNCHANGED);
    cv::Mat depth_img2 = cv::imread("../2_depth.png", cv::IMREAD_UNCHANGED);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts1, pts2;

    for (cv::DMatch m : matches)
    {
        ushort depth1 = depth_img1.at<ushort>(key_points1[m.queryIdx].pt);
        ushort depth2 = depth_img2.at<ushort>(key_points2[m.trainIdx].pt);
        if (depth1 == 0 || depth2 == 0)
            continue;
        cv::Point2d p1 = pixel2camera(key_points1[m.queryIdx].pt, K);
        cv::Point2d p2 = pixel2camera(key_points2[m.trainIdx].pt, K);
        double d1 = depth1 / 5000.0;    // 相机Z坐标轴上的单位1对应深度图的像素值为5000
        double d2 = depth2 / 5000.0;
        pts1.emplace_back(p1.x * d1, p1.y * d1, d1);
        pts2.emplace_back(p2.x * d2, p2.y * d2, d2);
    }
    std::cout << "3d-3d pairs: " << pts1.size() << std::endl;

    cv::Mat R, t;
    // 手写ICP
    pose_estimation_3d3d(pts1, pts2, R, t);
    std::cout << "ICP via SVD results: " << std::endl;
    std::cout << "R = " << R << std::endl;
    std::cout << "t = " << t << std::endl;

    // G2O优化
    bundleAdjustment(pts1, pts2, R, t);

    // verify p1 = R * p2 + t
    for (int i = 0; i < 5; i++)
    {
        std::cout << "p1 = " << pts1[i] << std::endl;
        std::cout << "p2 = " << pts2[i] << std::endl;
        std::cout << "(R * p2 + t) = " << R * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << std::endl;
    }

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


cv::Point2d pixel2camera(const cv::Point2d & p, const cv::Mat & K)
{
    return {(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)};
}


void pose_estimation_3d3d(
        const std::vector<cv::Point3f> & pts1,
        const std::vector<cv::Point3f> & pts2,
        cv::Mat & R,
        cv::Mat & t
)
{
    cv::Point3f p1, p2; // center of mass
    int N = (int) pts1.size();
    for (int i = 0; i < N; ++i)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 /= N;
    p2 /= N;

    std::vector<cv::Point3f> q1(N), q2(N);  // remove the center
    for (int i = 0; i < N; ++i)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    std::cout << "W = " << std::endl << W << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U, V;
    U = svd.matrixU();
    V = svd.matrixV();
    std::cout << "U = " << std::endl << U << std::endl;
    std::cout << "V = " << std::endl << V << std::endl;

    Eigen::Matrix3d R_eigen = U * V.transpose();
    if (R_eigen.determinant() < 0)
        R_eigen = -R_eigen;
    Eigen::Vector3d t_eigen = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_eigen * Eigen::Vector3d(p2.x, p2.y, p2.z);

    cv::eigen2cv(R_eigen, R);
    cv::eigen2cv(t_eigen, t);
}


class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    void oplusImpl(const double * update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    bool read(std::istream &in) override {}
    bool write(std::ostream &out) const override {}
};

class EdgeProjectRgbdPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
    explicit EdgeProjectRgbdPoseOnly(Eigen::Vector3d point) : point_(std::move(point)) {}

    void computeError() override
    {
        const auto v = dynamic_cast<const VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        _error = _measurement - T * point_;  // 取(u,v,1)中的前两维
    }

    void linearizeOplus() override
    {
        const auto v = dynamic_cast<const VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(T * point_);
    }

    bool read(std::istream &in) override {}
    bool write(std::ostream &out) const override {}

private:
    Eigen::Vector3d point_;
};


void bundleAdjustment(
        const std::vector<cv::Point3f> & pts1,
        const std::vector<cv::Point3f> & pts2,
        cv::Mat & R,
        cv::Mat & t
)
{
    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    auto vertex = new VertexPose();
    vertex->setId(0);
    vertex->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex);

    // edge
    for (int i = 0; i < pts1.size(); ++i)
    {
        auto edge = new EdgeProjectRgbdPoseOnly(Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        edge->setId(i + 1);
        edge->setVertex(0, vertex);
        edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    Eigen::Matrix3d R_eigen = vertex->estimate().rotationMatrix();
    Eigen::Vector3d t_eigen = vertex->estimate().translation();

    cv::eigen2cv(R_eigen, R);
    cv::eigen2cv(t_eigen, t);
}
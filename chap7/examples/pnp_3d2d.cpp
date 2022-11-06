/*
 * @Description: EPnP 3D-2D
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-29 15:01:37
 */

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>   // 注意cv2eigen头文件包含顺序
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
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


// BA by g2o
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
void bundleAdjustmentG2O(
        const VecVector3d & points_3d,
        const VecVector2d & points_2d,
        const cv::Mat & K,
        Sophus::SE3d & pose
);


// BA by gauss-newton
void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat & K,
        Sophus::SE3d &pose
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
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3d> pts_3d;
    std::vector<cv::Point2d> pts_2d;
    for (cv::DMatch m : matches)
    {
        ushort depth = depth_img1.at<ushort>(key_points1[m.queryIdx].pt);
        if (depth == 0)
            continue;
        double d = depth / 5000.0;   // 相机Z坐标轴上的单位1对应深度图的像素值为5000
        cv::Point2d p_camera = pixel2camera(key_points1[m.queryIdx].pt, K);
        pts_3d.emplace_back(p_camera.x * d, p_camera.y * d, d);
        pts_2d.push_back(key_points2[m.trainIdx].pt);
    }
    std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

    // OpenCV自带的PnP求解
    cv::Mat r, t;
    cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);    // 从旋转向量转换为矩阵
    std::cout << "R = " << std::endl << R << std::endl;
    std::cout << "t = " << std::endl << t << std::endl;

    // Mat -> Eigen
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (int i = 0; i < pts_3d.size(); ++i)
    {
        pts_3d_eigen.emplace_back(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z);
        pts_2d_eigen.emplace_back(pts_2d[i].x, pts_2d[i].y);
    }

    // Gauss-Newton BA求解
    Sophus::SE3d pose_gn;
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    std::cout << "pose by gauss-newton = " << std::endl << pose_gn.matrix() << std::endl;

    // G2O BA求解
    Sophus::SE3d pose_g2o;
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    std::cout << "pose by g2o = " << std::endl << pose_g2o.matrix() << std::endl;

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


void bundleAdjustmentGaussNewton(
        const VecVector3d & points_3d,
        const VecVector2d & points_2d,
        const cv::Mat & K,
        Sophus::SE3d & pose)
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost, last_cost;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; ++iter)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        for (int i = 0; i < points_3d.size(); ++i)
        {
            Eigen::Vector3d p_camera = pose * points_3d[i]; // 世界坐标系转相机坐标系

            double x = p_camera[0], y = p_camera[1], z = p_camera[2];
            Eigen::Vector2d p_pixel(fx * x / z + cx, fy * y / z + cy);  // 相机坐标系转像素坐标系

            Eigen::Vector2d error = points_2d[i] - p_pixel; // 重投影误差
            cost += error.squaredNorm();

            Eigen::Matrix<double, 2, 6> J;  // Jacobian

            J <<
                -fx / z,
                0,
                fx * x / (z * z),
                fx * x * y / (z * z),
                -fx - fx * x * x / (z * z),
                fx * y / z,
                0,
                -fy / z,
                fy * y / (z * z),
                fy + fy * y * y / (z * z),
                -fy * x * y / (z * z),
                -fy * x / z;

            H += J.transpose() * J;
            b += -J.transpose() * error;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (std::isnan(dx[0]))
        {
            std::cout << "result is NAN." << std::endl;
            break;
        }

        if (iter > 0 && cost >= last_cost)
            break;

        // update estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        last_cost = cost;

        if (dx.norm() < 1e-6)
            break;
    }
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


class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeProjection(Eigen::Vector3d pose, Eigen::Matrix3d K) : pose_(std::move(pose)), K_(std::move(K)) {}

    void computeError() override
    {
        const auto v = dynamic_cast<const VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pixel = K_ * (T * pose_);
        pixel /= pixel[2];
        _error = _measurement - pixel.head(2);  // 取(u,v,1)中的前两维
    }

    void linearizeOplus() override
    {
        const auto v = dynamic_cast<const VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d p_camera = T * pose_;
        double fx = K_(0, 0);
        double fy = K_(1, 1);
        double x = p_camera[0];
        double y = p_camera[1];
        double z = p_camera[2];
        _jacobianOplusXi <<
            -fx / z, 0, fx * x / (z * z), fx * x * y / (z * z), -fx - fx * x * x / (z * z), fx * y / z,
            0, -fy / z, fy * y / (z * z), fy + fy * y * y / (z * z), fy * x * y / (z * z), fy * x / z;
//        std::cout << "2" << std::endl;
    }

    bool read(std::istream &in) override {}
    bool write(std::ostream &out) const override {}

private:
    Eigen::Vector3d pose_;
    Eigen::Matrix3d K_;
};


void bundleAdjustmentG2O(
        const VecVector3d & points_3d,
        const VecVector2d & points_2d,
        const cv::Mat & K,
        Sophus::SE3d & pose
)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockServerType; // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockServerType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockServerType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    auto vertex = new VertexPose();
    vertex->setId(0);
    vertex->setEstimate(Sophus::SE3d());    // 手写GN里传入的pose
    optimizer.addVertex(vertex);

    // edge
    Eigen::Matrix3d K_eigen;
    cv::cv2eigen(K, K_eigen);
    for (int i = 0; i < points_2d.size(); ++i)
    {
        auto edge = new EdgeProjection(points_3d[i], K_eigen);
        edge->setId(i + 1);
        edge->setVertex(0, vertex);
        edge->setMeasurement(points_2d[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);
    pose = vertex->estimate();
}
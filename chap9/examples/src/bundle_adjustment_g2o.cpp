/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-07 11:34:59
 */

#include "common.h"

#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>

void solveBA(BALProblem & bal_problem);

class PoseAndIntrinsics
{
public:
    PoseAndIntrinsics() = default;

    explicit PoseAndIntrinsics(double * data)
    {
        rotation = Sophus::SO3d::exp(Eigen::Vector3d(data[0], data[1], data[2]));   // 旋转矩阵的李代数转成李群
        translation = Eigen::Vector3d(data[3], data[4], data[5]);
        focal = data[6];
        k1 = data[7];
        k2 = data[8];
    }

    // 将估计值放入内存
    void set_to_data(double * data)
    {
        auto r = rotation.log();    // 旋转矩阵的李群转成李代数
        for (int i = 0; i < 3; ++i)
        {
            data[i] = r[i];
            data[i + 3] = translation[i];
        }
        data[6] = focal;
        data[7] = k1;
        data[8] = k2;
    }

    Sophus::SO3d rotation;   // 旋转矩阵的李群
    Eigen::Vector3d translation;
    double focal = 0;
    double k1 = 0, k2 = 0;  // 畸变参数
};


class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override
    {
        _estimate = PoseAndIntrinsics();
    }

    void oplusImpl(const double * update) override
    {
        _estimate.rotation = Sophus::SO3d::exp(Eigen::Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Eigen::Vector3d(update[4], update[5], update[6]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    // 根据估计值投影一个点
    Eigen::Vector2d project(const Eigen::Vector3d & point)
    {
        Eigen::Vector3d p_camera = _estimate.rotation * point + _estimate.translation;  // 相机坐标系
        p_camera = -p_camera / p_camera[2]; // 归一化平面
        double r2 = p_camera.squaredNorm(); // x^2 + y^2
        double distortion = 1 + _estimate.k1 * r2 + _estimate.k1 * r2 * r2; // 畸变
        return {_estimate.focal * distortion * p_camera[0], _estimate.focal * distortion * p_camera[1]};
    }

    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}
};


class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override
    {
        _estimate = Eigen::Vector3d::Zero();
    }

    void oplusImpl(const double * update) override
    {
        _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
    }

    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}
};


class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseAndIntrinsics, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 计算曲线模型误差
    void computeError() override
    {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
        auto v1 = (VertexPoint *) (_vertices[1]);
        Eigen::Vector2d protection = v0->project(v1->estimate());
        _error = protection - _measurement;
    }

    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}
};


int main()
{
    std::string file_path = "../problem-16-22106-pre.txt";
    BALProblem bal_problem(file_path);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("../initial.ply");
    solveBA(bal_problem);
    bal_problem.WriteToPLYFile("../final.ply");

    return 0;
}


void solveBA(BALProblem & bal_problem)
{
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double * points = bal_problem.mutable_points();
    double * cameras = bal_problem.mutable_cameras();
    const double * observations = bal_problem.observations(); // [2 * n]

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    std::vector<VertexPoseAndIntrinsics *> vertex_poses_intrinsics;
    for (int i = 0; i < bal_problem.num_cameras(); ++i)
    {
        auto v = new VertexPoseAndIntrinsics;
        double * camera = cameras + camera_block_size * i;
        v->setId(i);
        v->setEstimate(PoseAndIntrinsics(camera));
        optimizer.addVertex(v);
        vertex_poses_intrinsics.push_back(v);
    }

    std::vector<VertexPoint *> vertex_points;
    for (int i = 0; i < bal_problem.num_points(); ++i)
    {
        auto v = new VertexPoint;
        double * point = points + point_block_size * i;
        v->setId(i + bal_problem.num_cameras());
        v->setEstimate(Eigen::Vector3d(point[0], point[1], point[2]));
        v->setMarginalized(true);   // 为稀疏优化设置边缘化顶点
        optimizer.addVertex(v);
        vertex_points.push_back(v);
    }

    for (int i = 0; i < bal_problem.num_observations(); ++i)
    {
        auto e = new EdgeProjection;
        e->setVertex(0, vertex_poses_intrinsics[bal_problem.camera_index()[i]]);
        e->setVertex(1, vertex_points[bal_problem.point_index()[i]]);
        e->setMeasurement(Eigen::Vector2d(observations[2 * i], observations[2 * i + 1]));
        e->setInformation(Eigen::Matrix2d::Identity());
        e->setRobustKernel(new g2o::RobustKernelHuber);
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(40);

    for (int i = 0; i < bal_problem.num_cameras(); ++i)
    {
        double * camera = cameras + camera_block_size * i;
        auto vertex = vertex_poses_intrinsics[i];
        auto estimate = vertex->estimate();
        estimate.set_to_data(camera);
    }
    for (int i = 0; i < bal_problem.num_points(); ++i)
    {
        double * point = points + point_block_size * i;
        auto vertex = vertex_points[i];
        for (int k = 0; k < 3; ++k)
            point[k] = vertex->estimate()[k];
    }
}
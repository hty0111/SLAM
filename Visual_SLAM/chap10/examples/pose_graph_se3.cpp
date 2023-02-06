/*
 * @Description: 用李代数优化位姿
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-07 22:37:26
 */

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <iostream>


typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


Matrix6d JRInv(const Sophus::SE3d & err)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = Sophus::SO3d::hat(err.so3().log());   // 变换矩阵error对应的旋转矩阵的李代数的反对称矩阵
    J.block(0, 3, 3, 3) = Sophus::SO3d::hat(err.translation()); // 变换矩阵error对应的平移向量的反对称矩阵
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
    J.block(3, 3, 3, 3) = Sophus::SO3d::hat(err.so3().log());
    J = Matrix6d::Zero() + 0.5 * J;
//    J = Matrix6d::Identity();   // 近似成单位阵
    return J;
}


class VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream & is) override
    {
        double data[7];
        for (double & i : data)
            is >> i;
        setEstimate(Sophus::SE3d(
                Eigen::Quaternion(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
                ));
    }

    bool write (std::ostream & os) const override
    {
        os << id() << " ";
        Eigen::Quaternion q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }

    void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    void oplusImpl(const double * update) override
    {
        Vector6d u;
        for (int i = 0; i < 6; ++i)
            u << update[i];
        _estimate = Sophus::SE3d::exp(u) * _estimate;
    }
};


class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream & is) override
    {
        double data[7];
        for (double & i : data)
            is >> i;
        Eigen::Quaternion q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(Sophus::SE3d(q, Eigen::Vector3d(data[0], data[1], data[2])));

        // TODO 设置信息矩阵？为啥
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool write (std::ostream & os) const override
    {
        auto v1 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[0]);
        auto v2 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";

        Sophus::SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++)
                os << information()(i, j) << " ";
        os << std::endl;

        return true;
    }

    void computeError() override
    {
        Sophus::SE3d v1 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[0])->estimate();    // Ti
        Sophus::SE3d v2 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[1])->estimate();    // Tj
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    void linearizeOplus() override
    {
        Sophus::SE3d v1 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[0])->estimate();
        Sophus::SE3d v2 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[1])->estimate();
        Matrix6d J = JRInv(Sophus::SE3d::exp(_error));
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};


int main()
{
    std::ifstream fin("../sphere.g2o");

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertex_cnt = 0, edge_cnt = 0;
    std::vector<VertexSE3LieAlgebra *> vertices;
    std::vector<EdgeSE3LieAlgebra *> edges;

    while (!fin.eof())
    {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT")
        {
            auto v = new VertexSE3LieAlgebra;
            int index;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertex_cnt++;
            if (index == 0)
                v->setFixed(true);  // 第一个点固定，不做优化
        }
        else if (name == "EDGE_SE3:QUAT")
        {
            auto e = new EdgeSE3LieAlgebra;
            int index1, index2;
            fin >> index1 >> index2;
            e->setId(edge_cnt++);
            e->setVertex(0, optimizer.vertices()[index1]);
            e->setVertex(1, optimizer.vertices()[index2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good())
            break;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(30);

    std::ofstream fout("../result_lie.g2o");
    for (auto v : vertices)
    {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (auto e : edges)
    {
        fout << "VERTEX_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();

    return 0;
}


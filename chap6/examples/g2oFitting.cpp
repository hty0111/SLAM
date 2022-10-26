/*
 * @Description: g2o图优化拟合曲线
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-25 14:38:09
 */

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <iostream>


// 曲线模型的顶点，模板参数：优化变量维度 & 数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重载reset函数
    void setToOriginImpl() override
    {
        _estimate << 0, 0, 0;
    }

    // 重载顶点的更新函数
    void oplusImpl(const double * update) override
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘，留空
    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}
};


// 曲线模型的边，即误差模型，模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 计算曲线模型误差
    void computeError() override
    {
        const auto v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error[0] = _measurement - exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
//        std::cout << "1" << std::endl;
    }

    // 计算雅克比矩阵
    void linearizeOplus () override
    {
        const auto v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
//        std::cout << "2" << std::endl;
    }

    // read & write
    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}

private:
    double _x;  // _y == _measurement
};


int main()
{
    double ar = 1.0, br = 2.0, cr = 1.0;    // real
    double ae = 2.0, be = -1.0, ce = 5.0;   // estimate
    int N = 100;    // data number
    double w_sigma = 1.0;    // 噪声标准差
    double w_sigma_inv = 1.0 / w_sigma;
    cv::RNG rng;

    // 生成数据
    std::vector<double> x_data, y_data;
    for (int i = 0; i < N; ++i)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));   // 带有噪声的测量值
    }

    /* 上面都一样 */

    // 构建图优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType; // 每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;   // 线性求解器类型

    // 梯度下降方法：GN/LM/GogLeg
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开输出

    // 往图中增加顶点
    auto v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // 往图中增加边
    for (int i = 0; i < N; ++i)
    {
        auto edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);  // 连接顶点
        edge->setMeasurement(y_data[i]);    // 观测值
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));    // 信息矩阵：协方差的逆
        optimizer.addEdge(edge);
    }

    // 开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(10); // 迭代次数

    // 输出结果
    std::cout << "estimated model: " << v->estimate().transpose() << std::endl;

    return 0;
}


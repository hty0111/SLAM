/**
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2023-02-04 19:14:29
 */

#include "g2o_optimize.h"

class PoseVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseVertex() = default;
    explicit PoseVertex(Eigen::Vector3d origin)
    {
        _origin = origin;
    }

    void setToOriginImpl() override
    {
        _estimate = _origin;
    }

    void oplusImpl(const double * update) override
    {
        _estimate += Eigen::Vector3d(update);
        _estimate(2) = normalizeAngle(_estimate(2));
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}

private:
    Eigen::Vector3d _origin;
};


class PoseEdge : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, PoseVertex, PoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseEdge() = default;

    void computeError() override
    {
        const auto * v0 = dynamic_cast<const PoseVertex *>(_vertices[0]);
        const auto * v1 = dynamic_cast<const PoseVertex *>(_vertices[1]);

        const Eigen::Vector3d xi = v0->estimate(), xj = v1->estimate(), z = _measurement;
        Eigen::Vector2d ti(xi(0), xi(1)), tj(xj(0), xj(1)), tij(z(0), z(1));
        double theta_i = xi(2), theta_j = xj(2), theta_ij = z(2);

        Eigen::Matrix2d Rij = PoseToTrans(z).block(0, 0, 2, 2);
        Eigen::Matrix2d Ri = PoseToTrans(xi).block(0, 0, 2, 2);
        Eigen::Matrix2d Rj = PoseToTrans(xj).block(0, 0, 2, 2);

        _error.block(0, 0, 2, 1) = Rij.transpose() * (Ri.transpose() * (tj - ti) - tij);
        _error(2) = normalizeAngle(theta_j - theta_i - theta_ij);
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}
};


void g2oOptimize(std::vector<Eigen::Vector3d> & Vertexs, std::vector<Edge> & Edges, int iter)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType; // 每个误差项优化变量维度为3，误差值维度为3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;   // 线性求解器类型

    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    std::vector<PoseVertex *> vertice;
    for (int i = 0; i < Vertexs.size(); ++i)
    {
        auto * v = new PoseVertex(Vertexs[i]);
        if (i == 0)
            v->setFixed(true);
        v->setEstimate(Vertexs[i]);
        v->setId(i);
        optimizer.addVertex(v);
        vertice.push_back(v);
    }

    for (int i = 0; i < Edges.size(); ++i)
    {
        auto * e = new PoseEdge();
        e->setId(i);
        e->setVertex(0, vertice[Edges[i].xi]);
        e->setVertex(1, vertice[Edges[i].xj]);
        e->setMeasurement(Edges[i].measurement);
        e->setInformation(Edges[i].infoMatrix);
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(iter);

    std::cout << "g2o optimization finished.";

    for (int i = 0; i < vertice.size(); ++i)
        Vertexs[i] = vertice[i]->estimate();
}


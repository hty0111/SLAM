/*
 * @Description: 用李群优化位姿（四元数）
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-07 22:37:44
 */

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <iostream>

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
    while (!fin.eof())
    {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT")
        {
            auto v = new g2o::VertexSE3;
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
            auto e = new g2o::EdgeSE3;
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
    optimizer.save("../result.g2o");

    return 0;
}
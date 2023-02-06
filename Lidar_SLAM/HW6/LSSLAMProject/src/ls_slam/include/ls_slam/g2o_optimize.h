/**
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2023-02-04 19:14:29
 */

//#include <g2o/core/g2o_core_api.h>
//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_binary_edge.h>
//#include <g2o/core/hyper_graph.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "gaussian_newton.h"

#ifndef LS_SLAM_G2O_H
#define LS_SLAM_G2O_H

void g2oOptimize(std::vector<Eigen::Vector3d> & Vertexs, std::vector<Edge> & Edges, int iter);

#endif //LS_SLAM_G2O_H

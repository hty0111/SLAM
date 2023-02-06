#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <iostream>


//位姿-->变换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}


//变换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}

// T-->R
Eigen::Matrix2d TransToRot(Eigen::Matrix3d trans)
{
    Eigen::Matrix2d rot;
    rot = trans.block(0, 0, 2, 2);
    return rot;
}

//计算整个pose-graph的误差
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for(const auto& edge : Edges)
    {
        Eigen::Vector3d xi = Vertexs[edge.xi];
        Eigen::Vector3d xj = Vertexs[edge.xj];
        Eigen::Vector3d z = edge.measurement;
        Eigen::Matrix3d infoMatrix = edge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
 */
void CalcJacobianAndError(const Eigen::Vector3d& xi, const Eigen::Vector3d& xj, const Eigen::Vector3d& z,
                          Eigen::Vector3d& ei, Eigen::Matrix3d& Ai, Eigen::Matrix3d& Bi)
{
    //TODO--Start
    Eigen::Vector2d ti(xi(0), xi(1)), tj(xj(0), xj(1)), tij(z(0), z(1));
    double theta_i = xi(2), theta_j = xj(2), theta_ij = z(2);

    Eigen::Matrix2d Rij = PoseToTrans(z).block(0, 0, 2, 2);
    Eigen::Matrix2d Ri = PoseToTrans(xi).block(0, 0, 2, 2);
    Eigen::Matrix2d Rj = PoseToTrans(xj).block(0, 0, 2, 2);

    ei.block(0, 0, 2, 1) = Rij.transpose() * (Ri.transpose() * (tj - ti) - tij);
    // 角度限位
    ei(2) = normalizeAngle(theta_j - theta_i - theta_ij);

    Eigen::Matrix2d dRiT;
    dRiT << -sin(theta_i), cos(theta_i), -cos(theta_i), -sin(theta_i);

    Ai.block(0, 0, 2, 2) = -Rij.transpose() * Ri.transpose();
    Ai.block(0, 2, 2, 1) = Rij.transpose() * dRiT * (tj - ti);
    Ai.block(2, 0, 1, 3) = Eigen::Vector3d(0, 0, -1).transpose();

    Bi.setIdentity();
    Bi.block(0, 0, 2, 2) = Rij.transpose() * Ri.transpose();

    //TODO--end
}

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    //构造H矩阵 & b向量
    for(const auto& edge : Edges)
    {
        //提取信息
        Eigen::Vector3d xi = Vertexs[edge.xi];
        Eigen::Vector3d xj = Vertexs[edge.xj];
        Eigen::Vector3d z = edge.measurement;
        Eigen::Matrix3d infoMatrix = edge.infoMatrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bi;
        CalcJacobianAndError(xi, xj, z, ei, Ai, Bi);

        //TODO--Start

        // 这写法效率差了很多
//        Eigen::MatrixXd J(3,Vertexs.size() * 3);
//        J.setZero();
//        J.block(0, edge.xi * 3, 3, 3) = Ai;
//        J.block(0, edge.xj * 3, 3, 3) = Bi;
//
//        H += J.transpose() * infoMatrix * J;
//        b += J.transpose() * infoMatrix * ei;

        H.block(edge.xi * 3, edge.xi * 3, 3, 3) += Ai.transpose() * infoMatrix * Ai;
        H.block(edge.xi * 3, edge.xj * 3, 3, 3) += Ai.transpose() * infoMatrix * Bi;
        H.block(edge.xj * 3, edge.xj * 3, 3, 3) += Bi.transpose() * infoMatrix * Bi;
        H.block(edge.xj * 3, edge.xi * 3, 3, 3) += Bi.transpose() * infoMatrix * Ai;
        b.block(edge.xi * 3, 0, 3, 1) += Ai.transpose() * infoMatrix * ei;
        b.block(edge.xj * 3, 0, 3, 1) += Bi.transpose() * infoMatrix * ei;

        //TODO--End
    }

    //求解
    Eigen::VectorXd dx(Vertexs.size() * 3);
    //TODO--Start

    dx.setZero();
    Eigen::SparseMatrix<double> A = H.sparseView();
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    if (solver.info() == Eigen::Success)
        dx = solver.solve(-b);

    //TODO-End

    return dx;
}


double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= M_PI * 2;
    while (angle < -M_PI)
        angle += M_PI * 2;
    return angle;
}









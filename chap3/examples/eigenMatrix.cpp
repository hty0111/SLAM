/*
 * @Description: Eigen矩阵模板类
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-20 11:16:29
 */

#include <iostream>
#include <ctime>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char ** argv)
{
    // 底层都是Matrix类型
    Eigen::Matrix<float, 2, 3> matrix_23;   // 2*3
    Eigen::Vector3d vector3D;   // 3*1
    Eigen::Matrix3d matrix3D = Eigen::Matrix3d::Zero();     // 初始化为0
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;   // 动态大小
    Eigen::MatrixXd matrixXd;   // 与上一行等价

    // 矩阵操作
    matrix_23 << 1,2,3,4,5,6;   // 重载了输入和输出
    cout << matrix_23(1, 2) << endl;    // 6
    vector3D << 3,2,1;
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * vector3D;   // float转double才能相乘
    matrix3D = Eigen::Matrix3d::Random();
    matrix3D.transpose();   // 转置
    matrix3D.sum();         // 所有元素的和
    matrix3D.trace();       // 迹
    matrix3D.inverse();     // 逆
    matrix3D.determinant(); // 行列式

    // 特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix3D.transpose() * matrix3D);
    eigen_solver.eigenvalues();     // 特征值
    eigen_solver.eigenvectors();    // 特征向量

    // 解方程 matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
    clock_t time_st = clock();
    // 直接求逆
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "Time use in normal inverse: " << 1000 * (clock() - time_st) / CLOCKS_PER_SEC << "ms" << endl;
    // QR分解
    time_st = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "Time use in QR composition: " << 1000 * (clock() - time_st) / CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}

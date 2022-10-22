/*
 * @Description: 取左上角3x3矩阵
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-22 20:32:37
 */

#include <Eigen/Core>
#include <iostream>

int main()
{
    Eigen::Matrix<double, 9, 9> matrix_99 = Eigen::MatrixXd::Random(9, 9);
    std::cout << "Original 9x9 matrix:" << std::endl << matrix_99.matrix() << std::endl << std::endl;

    // read method 1
    Eigen::Matrix3d matrix_33_1 = matrix_99.block(0, 0, 3, 3);
    std::cout << "Read method 1:" << std::endl << matrix_33_1.matrix() << std::endl << std::endl;

    // read method 2
    Eigen::Matrix3d matrix_33_2 = matrix_99.topLeftCorner(3, 3);
    std::cout << "Read method 2:" << std::endl << matrix_33_2.matrix() << std::endl << std::endl;

    // write
    Eigen::Matrix3d matrix_33_3 = Eigen::Matrix3d::Identity();
    matrix_99.block(0, 0, 3, 3) = matrix_33_3;
    std::cout << "Write to identity:" << std::endl << matrix_99.block(0, 0, 5, 5) << std::endl;

    return 0;
}


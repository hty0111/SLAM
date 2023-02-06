/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-22 20:32:46
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

int main()
{
    const int MATRIX_SIZE = 50;

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    using namespace std::chrono;
    high_resolution_clock::time_point start, end;

    // LU分解
    start = high_resolution_clock::now();
    matrix_NN.partialPivLu().solve(v_Nd);
//    matrix_NN.fullPivLu().solve(v_Nd);
    end = high_resolution_clock::now();
    std::cout << "LU consumes: " << (end - start).count() << " ms" << std::endl;

    // QR分解
    start = high_resolution_clock::now();
//    matrix_NN.householderQr().solve(v_Nd);
    matrix_NN.colPivHouseholderQr().solve(v_Nd);
//    matrix_NN.fullPivHouseholderQr().solve(v_Nd);
    end = high_resolution_clock::now();
    std::cout << "QR consumes: " << (end - start).count() << " ms" << std::endl;

    // Cholesky分解
    start = high_resolution_clock::now();
    matrix_NN.llt().solve(v_Nd);
//    matrix_NN.ldlt().solve(v_Nd);
    end = high_resolution_clock::now();
    std::cout << "Cholesky consumes: " << (end - start).count() << " ms" << std::endl;

    // SVD分解
//    start = high_resolution_clock::now();
//    matrix_NN.bdcSvd().solve(v_Nd);
//    matrix_NN.jacobiSvd().solve(v_Nd);
//    end = high_resolution_clock::now();
//    std::cout << "SVD consumes: " << (end - start).count() << " ms" << std::endl;

    return 0;
}


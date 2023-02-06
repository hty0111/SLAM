/*
 * @Description: 高斯牛顿法  y=exp(ax^2+bx+c)+w
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-24 19:52:25
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>


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

    // Gauss-Newton 迭代
    int iterations = 100;   // 迭代次数
    double cost, last_cost;

    for (int iter = 0; iter < iterations; ++iter)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();    // Hessian = J^T w^{-1}
        Eigen::Vector3d b = Eigen::Vector3d::Zero();    // bias
        cost = 0;

        for (int i = 0; i < N; ++i)
        {
            double xi = x_data[i], yi = y_data[i];
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Eigen::Vector3d J;  // Jacobian
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce); // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);      // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);           // de/dc

//            H += J * w_sigma_inv * w_sigma_inv * J.transpose(); // H = J w^{2}^{-1} J^T
//            b += -J * w_sigma_inv * w_sigma_inv * error;        // b = -J w^{2}^{-1} e
            H += w_sigma_inv * w_sigma_inv * J * J.transpose();
            b+= -w_sigma_inv * w_sigma_inv * error * J;

            cost += error * error;  // 均方误差
        }

        // 求解 Hx = b
        Eigen::Vector3d dx = H.ldlt().solve(b); // Cholesky分解
        if (isnan(dx[0]))
        {
            std::cout << "result is NAN." << std::endl;
            break;
        }

        if (iter > 0 && cost >= last_cost)  // 代价函数不减小，结束迭代
        {
            std::cout << "cost: " << cost << "\tlast cost: " << last_cost << std::endl;;
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        last_cost = cost;

        std::cout << "total cost: " << cost << "\tupdate: " << dx.transpose() << std::endl;
        std::cout << "estimated params: " << ae << " " << be << " " << ce << std::endl;
    }

    std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;

    return 0;
}

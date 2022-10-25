/*
 * @Description: ceres拟合曲线
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-24 20:52:47
 */

#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <iostream>


struct curve_fitting_cost_t
{
    curve_fitting_cost_t(double x, double y) : x_(x), y_(y) {}

    // 计算残差
    template<class T>
    bool operator()(const T * const abc,T * residual) const
    {
        residual[0] = T(y_) - ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);
        return true;
    }

    const double x_, y_;
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

    double abc[3] = {ae, be, ce};

    // 构造最小二乘
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<curve_fitting_cost_t, 1, 3>(new curve_fitting_cost_t(x_data[i], y_data[i])),
                nullptr,    // 不使用核函数
                abc // 待估计参数
                );
    }

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;    // 输出到stdout

    // 优化信息
    ceres::Solver::Summary summary;

    // 开始优化
    ceres::Solve(options, &problem, &summary);

    // 输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated abc = ";
    for (auto value : abc)
        std::cout << value << " ";
    std::cout << std::endl;

    return 0;
}

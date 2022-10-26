/*
 * @Description:
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-25 21:03:33
 */

#include <ceres/ceres.h>
#include <iostream>


struct F1
{
    template <class T>
    bool operator()(const T* const x1, const T* const x2, T* residual) const
    {
        // f1 = x1 + 10 * x2
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};


struct F2
{
    template <class T>
    bool operator()(const T* const x3, const T* const x4, T* residual) const
    {
        // f2 = sqrt(5) (x3 - x4)
        residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
        return true;
    }
};


struct F3
{
    template <class T>
    bool operator()(const T* const x2, const T* const x3, T* residual) const
    {
        // f3 = (x2 - 2 x3)^2
        residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
        return true;
    }
};


struct F4
{
    template <class T>
    bool operator()(const T* const x1, const T* const x4, T* residual) const
    {
        residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};


int main()
{
    double x1 = 3.0, x2 = -1.0, x3 =  0.0, x4 = 1.0;

    ceres::Problem problem;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, &x1, &x2);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x3, &x4);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x2, &x3);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, &x1, &x4);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    std::cout << "estimated params: " << x1 << " " << x2 << " " << x3 << " " << x4 << std::endl;

    return 0;
}


/*
 * @Description: ceres tutorials
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-10-25 19:42:25
 */

#include <ceres/ceres.h>
#include <iostream>

// y = 10 - x
struct CostFunctor
{
    template<class T>
    bool operator()(const T * const x, T * residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};


int main()
{
    double estimated_x = 5.0;   // init

    ceres::Problem problem;
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &estimated_x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated x: " << estimated_x << std::endl;

    return 0;
}
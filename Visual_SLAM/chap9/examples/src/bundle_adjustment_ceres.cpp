/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-07 11:35:14
 */

#include "common.h"
#include <SnavelyReprojectionError.h>
#include <ceres/ceres.h>
#include <iostream>

void solveBA(BALProblem & bal_problem);

int main()
{
    std::string file_path = "../problem-16-22106-pre.txt";
    BALProblem bal_problem(file_path);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("../initial.ply");
    solveBA(bal_problem);
    bal_problem.WriteToPLYFile("../final.ply");

    return 0;
}


void solveBA(BALProblem & bal_problem)
{
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double * points = bal_problem.mutable_points();
    double * cameras = bal_problem.mutable_cameras();
    const double * observations = bal_problem.observations(); // [2 * n]

    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i)
    {
        ceres::CostFunction * cost_function;
        // input = camera & point
        // output = pixel error
        cost_function = SnavelyReprojectionError::Create(observations[2 * i], observations[2 * i + 1]);

        // Huber核函数
        ceres::LossFunction * loss_function = new ceres::HuberLoss(1.0);

        // Each observation corresponds to a pair of a camera and a point
        // which are identified by camera_index()[i] and point_index()[i]
        // respectively.
        double * camera = cameras + camera_block_size * bal_problem.camera_index()[i];
        double * point = points + point_block_size * bal_problem.point_index()[i];

        problem.AddResidualBlock(cost_function, loss_function, camera, point);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-20 17:17:47
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <iostream>
#include <cmath>

int main()
{
    // 沿Z轴旋转90°的R
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q(R);    // 四元数
    Sophus::SO3d SO3_R(R);  // 通过旋转矩阵构造SO3
    Sophus::SO3d SO3_q(q);  // 通过四元数构造SO3
    std::cout << "Rotation matrix:" << std::endl << R << std::endl;
    std::cout << "SO3 from rotation matrix:" << std::endl << SO3_R.matrix() << std::endl;
    std::cout << "SO3 from quaternion:" << std::endl << SO3_q.matrix() << std::endl << std::endl;

    // 对数映射获得李代数
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3:" << std::endl << so3.transpose() << std::endl;
    std::cout << "so3 hat:" << std::endl << Sophus::SO3d::hat(so3) << std::endl;
    std::cout << "so3 hat vee:" << std::endl << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // 增量扰动模型
    Eigen::Vector3d so3_delta(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(so3_delta) * SO3_R;
    std::cout << "SO3 updated:" << std::endl << SO3_updated.matrix() << std::endl << std::endl;

    std::cout << "*************************" << std::endl << std::endl;

    Eigen::Vector3d t(1, 0, 0); // 沿x轴平移1
    Sophus::SE3d SE3_Rt(R, t);  //通过R,t构造SE3
    Sophus::SE3d SE3_qt(q, t);  // 通过q,t构造SE3
    std::cout << "SE3 from R,t:" << std::endl << SE3_Rt.matrix() << std::endl;
    std::cout << "SE3 from q,t:" << std::endl << SE3_qt.matrix() << std::endl;

    Eigen::Matrix<double, 6, 1> se3 = SE3_Rt.log();
    std::cout << "se3:" << std::endl << se3.transpose() << std::endl;
    std::cout << "se3 hat:" << std::endl << Sophus::SE3d::hat(se3) << std::endl;
    std::cout << "se3 hat vee:" << std::endl << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    Eigen::Matrix<double, 6, 1> se3_delta;
    se3_delta.setZero();
    se3_delta(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(se3_delta) * SE3_Rt;
    std::cout << "SE3 updated:" << std::endl << SE3_updated.matrix() << std::endl;

    return 0;
}

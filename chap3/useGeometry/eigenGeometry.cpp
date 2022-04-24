/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-20 14:06:18
 */

#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main()
{
    // 旋转矩阵R
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();  // 用单位矩阵初始化

    // 旋转向量Angle-Axis，底层不是Matrix，但可以用矩阵运算
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 沿z轴旋转45°
    rotation_vector.matrix();   // 转换为R
    rotation_vector.toRotationMatrix(); // 同上

    // 欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);    // ZYX

    // 变换矩阵T
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);  // 按照rotation_vector旋转
    T.pretranslate(Eigen::Vector3d(1, 3, 4));  // 把平移向量设成(1,3,4)

    // 四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector); // Angle-Axis赋给四元数
    q = Eigen::Quaterniond(rotation_matrix);    // R赋给四元数

    return 0;
}

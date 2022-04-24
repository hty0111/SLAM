/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-20 17:17:47
 */

#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main()
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 so3_R(R);
    Sophus::SO3 so3_V(0, 0, M_PI/2);
    Eigen::Quaterniond q(R);
    Sophus::SO3 so3_Q(q);

    cout << "SO3 from R:" << so3_R << endl;
    cout << "SO3 from V:" << so3_V << endl;
    cout << "SO3 from Q:" << so3_Q << endl;

    return 0;
}

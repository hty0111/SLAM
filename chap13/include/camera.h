/*
 * @Description: 双目相机
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-10 16:37:21
 */

#ifndef SLAM_CAMERA_H
#define SLAM_CAMERA_H

#include "common.h"

class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
    Matrix33 K;

    SE3 pose_;  // 从双目到单独的相机坐标系

    Camera() = default;
    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 & pose)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
    {
        K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    }

    Vector3d world2camera(const Vector3d &Pw, const SE3 &Tcw);
    Vector3d camera2world(const Vector3d &Pc, const SE3 &Tcw);
    Vector2d camera2pixel(const Vector3d &Pc);
    Vector3d pixel2camera(const Vector2d &Pp, double depth = 1);
    Vector2d world2pixel(const Vector3d &Pw, const SE3 &Tcw);
    Vector3d pixel2world(const Vector2d &Pp, const SE3 &Tcw, double depth = 1);
};

#endif //SLAM_CAMERA_H

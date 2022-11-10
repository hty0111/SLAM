/*
 * @Description: 双目相机
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-10 16:37:21
 */

#include "camera.h"

Vector3d Camera::world2camera(const Vector3d &Pw, const SE3 &Tcw)
{
    return pose_ * Tcw * Pw;
}

Vector3d Camera::camera2world(const Vector3d &Pc, const SE3 &Tcw)
{
    return Tcw.inverse() * pose_.inverse() * Pc;
}

Vector2d Camera::camera2pixel(const Vector3d &Pc)
{
    return Vector2d(fx_ * Pc(0, 0) / Pc(2, 0) + cx_, fy_ * Pc(1, 0) / Pc(2, 0) + cy_);
}

Vector3d Camera::pixel2camera(const Vector2d &Pp, double depth)
{
    return Vector3d((Pp(0, 0) - cx_) * depth / fx_, (Pp(1, 0) - cy_) * depth / fy_, depth);
}

Vector2d Camera::world2pixel(const Vector3d &Pw, const SE3 &Tcw)
{
    return camera2pixel(world2camera(Pw, Tcw));
}

Vector3d Camera::pixel2world(const Vector2d &Pp, const SE3 &Tcw, double depth)
{
    return camera2world(pixel2camera(Pp, depth), Tcw);
}

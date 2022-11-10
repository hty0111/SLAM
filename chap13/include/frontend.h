/*
 * @Description: 前端
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-10 16:30:05
 */

#ifndef SLAM_FRONTEND_H
#define SLAM_FRONTEND_H

#include "common.h"
#include "frame.h"
#include "map.h"
#include "camera.h"

#include <opencv2/features2d.hpp>


class Frontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    enum frontend_status{INIT, TRACKING_GOOD, TRACKING_BAD, LOST};
    frontend_status status_ = INIT;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_left_ = nullptr;   // 左侧相机
    Camera::Ptr camera_right_ = nullptr;  // 右侧相机
};

#endif //SLAM_FRONTEND_H

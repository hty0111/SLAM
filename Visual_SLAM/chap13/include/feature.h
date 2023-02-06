/*
 * @Description: 特征点
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 21:23:12
 */

#ifndef SLAM_FEATURE_H
#define SLAM_FEATURE_H

#include "common.h"
#include <opencv2/features2d.hpp>

struct Frame;
struct MapPoint;

/**
 * 2D特征，三角化后关联一个3D地图点
 */
struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;    // 持有该特征的帧，用weak_ptr防止shared_ptr循环引用
    cv::KeyPoint kp_;     // 2D位置
    std::weak_ptr<MapPoint> map_point_; // 关联地图点

    bool is_outlier_ = false;       // 是否为异常点
    bool is_on_left_image = true;   // 是否在左图

    Feature() = default;
    Feature(const std::shared_ptr<Frame> & frame, const cv::KeyPoint & kp) : frame_(frame), kp_(kp) {}
};


#endif //SLAM_FEATURE_H

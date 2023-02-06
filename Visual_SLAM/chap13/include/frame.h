/*
 * @Description: 帧
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 17:27:55
 */

#ifndef SLAM_FRAME_H
#define SLAM_FRAME_H

#include <utility>

#include "common.h"

struct MapPoint;    // TODO
struct Feature;

/**
 * 每一帧分配独立id，关键帧分配关键帧id
 */
struct Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_fey_frame_ = false; // 是否为关键帧
    SE3 pose_;  // Tcw, world to camera
    std::mutex mutex_;
    cv::Mat left_image_, right_image_;
    std::vector<std::shared_ptr<Feature>> features_left_, features_right_;  // 一帧里左右两幅图像的特征点

    Frame() = default;
    Frame(long id, const SE3 & pose, const cv::Mat & left, const cv::Mat & right) :
    id_(id), pose_(pose), left_image_(left), right_image_(right) {}

    SE3 SetPose(const SE3 & pose)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        pose_ = pose;
    }

    SE3 GetPose()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return pose_;
    }

    // 工厂模式，分配id
    static Frame::Ptr CreateFrame();

    // 设置关键帧并分配id
    void SetKeyFrame();
};

#endif //SLAM_FRAME_H

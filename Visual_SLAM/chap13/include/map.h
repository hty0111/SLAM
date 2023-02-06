/*
 * @Description: 地图
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-10 12:34:48
 */

#ifndef SLAM_MAP_H
#define SLAM_MAP_H

#include "common.h"
#include "frame.h"
#include "mappoint.h"

/**
 * 前端插入帧和地图点，后端剔除异常点，维护地图结构
 */

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;

    Map() = default;

    // 添加关键帧
    void InsertKeyFrame(const Frame::Ptr & frame);
    // 添加地图顶点
    void InsertMapPoint(const MapPoint::Ptr & mappoint);

    // 获取所有关键帧
    KeyframesType GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return keyframes_;
    }

    // 获取已激活关键帧
    KeyframesType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return active_keyframes_;
    }

    // 获取所有地图点
    LandmarksType GetAllLandmarks()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return landmarks_;
    }

    // 获取已激活地图点
    LandmarksType GetActiveLandmarks()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return active_landmarks_;
    }

    // 清理map中观测数量为0的点
    void CleanMap();

private:
    std::mutex mutex_;
    KeyframesType keyframes_;         // 所有关键帧
    KeyframesType active_keyframes_;  // 已激活关键帧，对应滑动窗口，即后端进行优化的部分
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;

    Frame::Ptr current_frame_ = nullptr;
    int active_keyframes_num_ = 7;

    // 保留最新的7个关键帧，将旧的关键帧设为不激活状态
    void RemoveOldKeyFrame();
};

#endif //SLAM_MAP_H

/*
 * @Description: 路标点
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 21:45:44
 */

#ifndef SLAM_MAPPOINT_H
#define SLAM_MAPPOINT_H

#include "common.h"
#include "feature.h"

struct Frame;
struct Feature;

/**
 * 特征点在三角化后形成路标点
 */
struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;
    bool is_outlier_ = false;
    Eigen::Vector3d point_w_ = Vector3d::Zero(); // 世界坐标系下的点
    std::mutex mutex_;
    int observed_times = 0; // 被特征匹配观察到的次数
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() = default;
    MapPoint(long id, Vector3d point) : id_(id), point_w_(point) {}

    Vector3d GetPointW()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return point_w_;
    }

    Vector3d SetPointW(const Vector3d & point_w)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        point_w_ = point_w;
    }

    void AddObservation(const std::shared_ptr<Feature> & feature);
    void RemoveObservation(const std::shared_ptr<Feature> & feature);

    // factory
    static MapPoint::Ptr CreateMapPoint();
};

#endif //SLAM_MAPPOINT_H

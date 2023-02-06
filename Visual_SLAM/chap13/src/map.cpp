/*
 * @Description: 地图
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-10 12:34:48
 */

#include "map.h"

void Map::InsertKeyFrame(const Frame::Ptr & frame)
{
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())   // 该帧不在已有关键帧中
    {
        keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    }
    else
    {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > active_keyframes_num_)
        RemoveOldKeyFrame();
}


void Map::InsertMapPoint(const MapPoint::Ptr & mappoint)
{
    if (landmarks_.find(mappoint->id_) == landmarks_.end())
    {
        landmarks_.insert(std::make_pair(mappoint->id_, mappoint));
        active_landmarks_.insert(std::make_pair(mappoint->id_, mappoint));
    }
    else
    {
        landmarks_[mappoint->id_] = mappoint;
        active_landmarks_[mappoint->id_] = mappoint;
    }
}


void Map::RemoveOldKeyFrame()
{
    if (current_frame_ == nullptr)
        return;

    // 寻找与当前帧最近和最远的两帧
    double min_dist = 999999, max_dist = 0;
    double min_dist_id = 0, max_dist_id = 0;
    auto Twc = current_frame_->GetPose().inverse();    // 世界坐标系下的相机位姿
    for (auto & kf : keyframes_)
    {
        if (kf.second == current_frame_)
            continue;
        auto dist = (kf.second->GetPose() * Twc).log().norm();  // Tcw * Twc 的李代数的范数
        if (dist > max_dist)
        {
            max_dist = dist;
            max_dist_id = kf.first;
        }
        if (dist < min_dist)
        {
            min_dist = dist;
            min_dist_id = kf.first;
        }
    }

    const double min_dist_th = 0.2; // 最近距离的阈值
    Frame::Ptr frame_to_move = nullptr;
    if (min_dist < min_dist_th)
        frame_to_move = keyframes_.at(min_dist_id);
    else
        frame_to_move = keyframes_.at(max_dist_id);

    LOG(INFO) << "remove keyframe " << frame_to_move->keyframe_id_;

    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_move->keyframe_id_);
    for (auto feature : frame_to_move->features_left_)
    {
        auto mp = feature->map_point_.lock();
        if (mp)
            mp->RemoveObservation(feature);
    }

    CleanMap();
}


void Map::CleanMap()
{
    int landmark_removed_num = 0;
    for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();)
    {
        if (iter->second->observed_times == 0)
        {
            active_landmarks_.erase(iter);  // erase后iter自动指向后一个
            landmark_removed_num++;
        }
        else
            iter++;
    }
    LOG(INFO) << "Removed " << landmark_removed_num << " active landmarks";
}




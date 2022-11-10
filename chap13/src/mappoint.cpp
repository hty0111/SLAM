/*
 * @Description: 路标点
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 21:45:44
 */

#include "mappoint.h"

void MapPoint::AddObservation(const std::shared_ptr<Feature> & feature)
{
    std::unique_lock<std::mutex> lock(mutex_);
    observations_.push_back(feature);
    observed_times++;
}


void MapPoint::RemoveObservation(const std::shared_ptr<Feature> & feature)
{
    std::unique_lock<std::mutex> lock(mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end(); ++iter)
    {
        if (iter->lock() == feature)
        {
            observations_.erase(iter);
            feature->map_point_.reset();
            observed_times--;
            break;
        }
    }
}


MapPoint::Ptr MapPoint::CreateMapPoint()
{
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}


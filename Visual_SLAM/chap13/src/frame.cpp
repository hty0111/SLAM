/*
 * @Description: 帧
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 17:27:55
 */

#include "frame.h"

Frame::Ptr Frame::CreateFrame()
{
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame()
{
    static long keyframe_factory_id = 0;
    is_fey_frame_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

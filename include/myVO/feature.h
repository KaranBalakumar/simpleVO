#pragma once

#ifndef MYVO_FEATURE_H
#define MYVO_FEATURE_H

#include <opencv2/features2d.hpp>
#include <memory>
#include "myVO/common_include.h"

namespace myVO{

struct Frame;
struct MapPoint;

struct Feature{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_;

    bool is_outlier_ = false;
    bool is_on_left_image_ = true;

    Feature(){}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp): frame_(frame), position_(kp){}

};

} // namespace myVO

#endif // MYVO_FEATURE_H
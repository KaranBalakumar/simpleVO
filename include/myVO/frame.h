#pragma once

#ifndef MYVO_FRAME_H
#define MYVO_FRAME_H

#include "myVO/camera.h"
#include "myVO/common_include.h"


namespace myVO {
    
// forward declare
struct MapPoint;
struct Feature;

struct Frame {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0; // frame id
    unsigned long keyframe_id_ = 0; // keyframe id
    bool is_keyframe_ = false; // is this frame a keyframe??
    double time_stamp_; // frame time_stamp(different from the dataset's timestamp)
    SE3 pose_; // Pose of the frame in CW
    std::mutex pose_mutex_; // Pose mutex
    cv::Mat left_img_, right_img_; // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    //corresponding features in right image, set to nullptr if no corresponsding features
    std::vector<std::shared_ptr<Feature>> features_right_;

    public: // data members
    Frame(){}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

    //set and get pose, thread safe
    SE3 Pose(){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    void SetKeyFrame();

    static std::shared_ptr<Frame> CreateFrame();

};

}

#endif // MYVO_FRAME_H
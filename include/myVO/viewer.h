#ifndef MYVO_VIEWER_H
#define MYVO_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "myVO/common_include.h"
#include "myVO/frame.h"
#include "myVO/map.h"

namespace myVO
{
    
/**
 * Visualization
 */

class Viewer{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map){map_= map;}

    void Close();

    // Add the current frame
    void AddCurrentFrame(Frame::Ptr current_frame);

    // Update Map
    void UpdateMap();

    private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

    // plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    std::unordered_map<unsigned long, MapPoint::Ptr> all_landmarks_;
    std::unordered_map<unsigned long, Frame::Ptr> all_keyframes_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};

} // namespace myVO

#endif // MYVO_VIEWER_H

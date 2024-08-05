#ifndef MYVO_BACKEND_H
#define MYVO_BACKEND_H

#include "myVO/common_include.h"
#include "myVO/frame.h"
#include "myVO/map.h"

namespace myVO
{
    
class Map;

/**
 * Backend
 * Has a separate optimization thread that starts optimization when the Map is updated
 * Map updates are triggered by the frontend
 */

class Backend{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    // Start the optimization thread and suspend it in the constructor
    Backend();

    // Set the left and right cameras to obtain internal and external parameters
    void SetCameras(Camera::Ptr left, Camera::Ptr right){
        cam_left_ = left;
        cam_right_ = right;
    }

    // Set up the map
    void SetMap(std::shared_ptr<Map> map){ map_ = map;}

    // Trigger the map update and start optimization
    void UpdateMap();

    // close the backend thread
    void Stop();

    private:
    // Backend thread
    void BackendLoop();
    
    // Optimize for the given keyframes and landmark points
    void Optimize(Map::KeyframesType &keyframe, Map::LandmarksType &landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

};

} // namespace myVO

#endif // MYVO_BACKEND_H

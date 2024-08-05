#ifndef MYVO_DATASET_H
#define MYVO_DATASET_H

#include "myVO/camera.h"
#include "myVO/common_include.h"
#include "myVO/frame.h"

namespace myVO
{

/**
 * Dataset Reading
 * Pass the configuration file path during construction; the dataset_dir in the config file specifies the dataset path.
 * After initialization, you can obtain the camera and the next frame image.
 */

class Dataset{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string &dataset_path);

    // Initialize, returns wether it was succesful
    bool Init();

    // Create and return the next frame containing he stereo images
    Frame::Ptr NextFrame();

    // get camera by id
    Camera::Ptr GetCamera(int camera_id) const{
        return cameras_.at(camera_id);
    }

    private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
    
} // namespace myVO

#endif //MYVO_DATASET_H

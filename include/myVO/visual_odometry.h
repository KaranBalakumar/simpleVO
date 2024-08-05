#pragma once
#ifndef MYVO_VISUAL_ODOMETRY_H
#define MYVO_VISUAL_ODOMETRY_H

#include "myVO/common_include.h"
#include "myVO/backend.h"
#include "myVO/dataset.h"
#include "myVO/frontend.h"
#include "myVO/viewer.h"

namespace myVO{
    
/**
 * VO External Interface
 */
class VisualOdometry{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    // constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    // Get Frontend status
    FrontendStatus GetFrontEndStatus() const { return frontend_->GetStatus();}
    

    private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    //dataset
    Dataset::Ptr dataset_ = nullptr;
};    
    
} // namespace myVo

#endif // MYVO_VISUAL_ODOMETRY_H

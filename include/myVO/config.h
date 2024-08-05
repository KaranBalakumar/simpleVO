#pragma once
#ifndef MYVO_CONFIG_H
#define MYVO_CONFIG_H

#include "common_include.h"

namespace myVO
{
    
class Config{
    private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config(){}
    public:
    ~Config(); // close the file when deconstructing;

    //set new config file
    static bool SetParameterFile(const std:: string &filename);

    //access the parameter values
    template <typename T>
    static T Get(const std:: string &key){
        return T(Config::config_->file_[key]);
    }
};

} // namespace myVO

#endif // MYVO_CONFIG_H
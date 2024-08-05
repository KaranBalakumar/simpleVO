#include <gflags/gflags.h>
#include "myVO/visual_odometry.h"

DEFINE_string(config_file, "/home/karan/simpleVO/config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myVO::VisualOdometry::Ptr vo(
        new myVO::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}

add_executable(run_kitti_stereo run_kitti_stereo.cpp)
target_link_libraries(run_kitti_stereo myVO ${THIRD_PARTY_LIBS})

include_directories("/usr/include/eigen3")

add_executable(viewPose viewPose.cpp)
target_link_libraries(viewPose myVO ${THIRD_PARTY_LIBS})
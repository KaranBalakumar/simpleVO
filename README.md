# simpleVO 
This repository is an example for using g2o to run visual odometry on KITTI dataset in C++

### Installation Dependencies
1. pangolin: <https://github.com/stevenlovegrove/Pangolin>
2. opencv4 (build from source is preferred)
3. Eigen (sudo apt install libeigen3-dev)
4. g2o: <https://github.com/RainerKuemmerle/g2o>

### Compiling the Code

```c++
mkdir build 
cd build
cmake ..
make
```

### Running the Code
```c++
cd build
../bin/run_kitti_stereo
```
Make sure you change the default.yaml file in config folder!

![simpleVO](doc/simpleVO.gif)

## Things to do
- Add loop closure with Bag of Words(DBoW3)
- Visualize the detected sparse featurepoints in the Pangolin Viewer 

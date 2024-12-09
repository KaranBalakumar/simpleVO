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
./run_kitti_stereo
```
Make sure you change the default.yaml file in config folder!

### Visualizing the Odometry
```c++
./viewPose
```
The odometry data will be written to pose.txt file in result directory

## please wait for the gif to load...

![simpleVO](doc/simpleVO.gif)

## Things to do
- Use ORB features
- Add loop closure thread with Bag of Words(DBoW3)
- Visualize the detected sparse featurepoints in the Pangolin Viewer [DONE]

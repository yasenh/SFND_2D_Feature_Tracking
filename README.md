# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.



## Benchmark

#### Number of Key-points for 10 Images

| Detectors | Number of Key-points |
| :-------: | :------------------: |
| SHITOMASI |        13423         |
|  HARRIS   |         1737         |
|   FAST    |        17874         |
|   BRISK   |        27116         |
|    ORB    |         5000         |
|   AKAZE   |        13429         |
|   SIFT    |                      |



#### Number of Matched Key-points for 10 Images

| Detectors\Descriptors | BRISK | BRIEF | ORB  | FREAK | AKAZE | SIFT |
| :-------------------: | :---: | :---: | :--: | :---: | :---: | :--: |
|       SHITOMASI       |  347  |  413  | 398  |  341  |       |      |
|        HARRIS         |  141  |  206  | 162  |  144  |       |      |
|         FAST          |  281  |  336  | 332  |  295  |       |      |
|         BRISK         |  276  |  314  | 266  |  292  |       |      |
|          ORB          |  339  |  267  | 347  |  327  |       |      |
|         AKAZE         |  349  |  392  | 345  |  353  |  343  |      |
|         SIFT          |       |       |      |       |       |      |



#### Key-point Detection and Descriptor Extraction Time Consumption (in ms)

| Detectors\Descriptors |  BRISK  |  BRIEF  |   ORB   |  FREAK  |  AKAZE  | SIFT |
| :-------------------: | :-----: | :-----: | :-----: | :-----: | :-----: | :--: |
|       SHITOMASI       | 98.8398 | 82.6777 | 91.0227 | 328.525 |         |      |
|        HARRIS         | 106.512 | 96.1124 | 108.656 | 338.423 |         |      |
|         FAST          | 12.7961 | 9.92533 | 12.1023 | 267.232 |         |      |
|         BRISK         | 262.799 | 257.95  | 262.838 | 510.137 |         |      |
|          ORB          | 53.0014 | 52.4011 | 58.3677 | 294.063 |         |      |
|         AKAZE         | 387.531 | 383.136 | 378.456 | 584.215 | 753.823 |      |
|         SIFT          |         |         |         |         |         |      |



#### Efficiency (matches/ms)

| Detectors\Descriptors |  BRISK   |  BRIEF  |   ORB    |  FREAK   |  AKAZE   | SIFT |
| :-------------------: | :------: | :-----: | :------: | :------: | :------: | :--: |
|       SHITOMASI       | 3.51073  | 4.9953  | 4.37254  | 1.03797  |          |      |
|        HARRIS         |  1.3238  | 2.14332 | 1.49094  | 0.425504 |          |      |
|         FAST          | 21.9598  | 33.8528 | 27.4329  | 1.10391  |          |      |
|         BRISK         | 1.05023  | 1.21729 | 1.01203  | 0.572395 |          |      |
|          ORB          | 6.39606  | 5.09532 | 5.94507  | 1.11201  |          |      |
|         AKAZE         | 0.900572 | 1.02314 | 0.911598 | 0.60423  | 0.455014 |      |
|         SIFT          |          |         |          |          |          |      |
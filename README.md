# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 



## Rubric

1. Data Buffer Optimization: Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.
2. Keypoint Detection: Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
3. Keypoint Removal: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
4. Keypoint Descriptors: Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
5. Descriptor Matching: Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
6. Descriptor Distance Ratio: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
7. Performance Evaluation 1: Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
8. Performance Evaluation 2: Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
9. Performance Evaluation 3: Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

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
|   SIFT    |        13862         |



#### Number of Matched Key-points for 10 Images

| Detectors\Descriptors | BRISK |  BRIEF  |      ORB      | FREAK | AKAZE | SIFT |
| :-------------------: | :---: | :-----: | :-----------: | :---: | :---: | :--: |
|       SHITOMASI       |  347  | **413** |      398      |  341  |  N/A  | 405  |
|        HARRIS         |  141  |   206   |      162      |  144  |  N/A  | 163  |
|         FAST          |  281  |   336   |      332      |  295  |  N/A  | 291  |
|         BRISK         |  276  |   314   |      266      |  292  |  N/A  | 279  |
|          ORB          |  339  |   267   |      347      |  327  |  N/A  | 364  |
|         AKAZE         |  349  |   392   |      345      |  353  |  343  | 348  |
|         SIFT          |  201  |   249   | Out of Memory |  195  |  N/A  | 294  |

* KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints.
* SHITOMASI with blockSize = 4
* HARRIS with blockSize = 2
* In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.



#### Key-point Detection and Descriptor Extraction Time Consumption (in ms)

| Detectors\Descriptors |  BRISK  |    BRIEF    |      ORB      |  FREAK  |  AKAZE  |    SIFT    |
| :-------------------: | :-----: | :---------: | :-----------: | :-----: | :-----: | :--------: |
|       SHITOMASI       | 98.8398 |   82.6777   |    91.0227    | 328.525 |   N/A   |  180.6775  |
|        HARRIS         | 106.512 |   96.1124   |    108.656    | 338.423 |   N/A   |  151.436   |
|         FAST          | 12.7961 | **9.92533** |    12.1023    | 267.232 |   N/A   |  116.5025  |
|         BRISK         | 262.799 |   257.95    |    262.838    | 510.137 |   N/A   | 337.020527 |
|          ORB          | 53.0014 |   52.4011   |    58.3677    | 294.063 |   N/A   |  270.4355  |
|         AKAZE         | 387.531 |   383.136   |    378.456    | 584.215 | 753.823 |  458.371   |
|         SIFT          | 607.335 |   623.61    | Out of Memory | 805.025 |   N/A   |  1072.55   |



#### Efficiency (matches/ms)

| Detectors\Descriptors |  BRISK   |    BRIEF    |      ORB      |  FREAK   |  AKAZE   |   SIFT   |
| :-------------------: | :------: | :---------: | :-----------: | :------: | :------: | :------: |
|       SHITOMASI       | 3.51073  |   4.9953    |    4.37254    | 1.03797  |   N/A    | 2.24156  |
|        HARRIS         |  1.3238  |   2.14332   |    1.49094    | 0.425504 |   N/A    | 1.07636  |
|         FAST          | 21.9598  | **33.8528** |    27.4329    | 1.10391  |   N/A    | 2.49780  |
|         BRISK         | 1.05023  |   1.21729   |    1.01203    | 0.572395 |   N/A    | 0.827842 |
|          ORB          | 6.39606  |   5.09532   |    5.94507    | 1.11201  |   N/A    | 1.34597  |
|         AKAZE         | 0.900572 |   1.02314   |   0.911598    | 0.60423  | 0.455014 | 0.759210 |
|         SIFT          | 0.330954 |  0.399288   | Out of Memory | 0.242228 |   N/A    | 0.274113 |



## TOP3 detector / descriptor combinations

1. FAST + BRIEF
2. FAST + ORB
3. FAST + BRISK



## References

1. [KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints](<https://github.com/kyamagu/mexopencv/issues/351>)

   
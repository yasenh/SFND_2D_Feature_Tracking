/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"


/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    // visualize results
    bool bVis = true;
    bool bVisKeyPoints = false;
    bool bFocusOnVehicle = true;
    bool bLimitKpts = true;

    double tTotal = 0;
    int totalKeyPointsNum = 0;
    int totalMatchesNum = 0;

    // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    std::string detectorType = "SHITOMASI";

    // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    std::string descriptorType = "BRISK";

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename;
        imgFullFilename.append(imgBasePath).append(imgPrefix).append(imgNumber.str()).append(imgFileType);

        // load image from file and convert to gray-scale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        // pop old images
        if(dataBuffer.size() > dataBufferSize) {
            dataBuffer.erase(dataBuffer.begin());
        }

        std::cout << std::endl;

        //// EOF STUDENT ASSIGNMENT
        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image



        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        // Apply corner detection
        double t1 = 0;

        if ("SHITOMASI" == detectorType) {
            t1 = detKeypointsShiTomasi(keypoints, imgGray);
        }
        else if ("HARRIS" == detectorType){
            t1 = detKeypointsHarris(keypoints, imgGray);
        }
        else {
            // FAST, BRISK, ORB, AKAZE, SIFT
            t1 = detKeypointsModern(keypoints, imgGray, detectorType);
        }

        totalKeyPointsNum += keypoints.size();

        // visualize results
        if (bVisKeyPoints) {
            cv::Mat visImage = img.clone();
            cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            std::string windowName = detectorType + " Detector Results";
            cv::namedWindow(windowName, 6);
            imshow(windowName, visImage);
            cv::waitKey(0);
        }

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle

        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle) {
            for (auto it = keypoints.begin(); it != keypoints.end();) {
                // remove the kep-point if it is not in the RoI
                if (!vehicleRect.contains(it->pt)) {
                    keypoints.erase(it);
                }
                else {
                    it++;
                }
            }
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        if (bLimitKpts) {
            int maxKeypoints = 50;
            // there is no response info, so keep the first 50 as they are sorted in descending quality order
            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << "---NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        // Descriptor is a vector of values, which describes the image patch around a keypoint
        cv::Mat descriptors;

        double t2 = 0;
        t2 = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

        // wait until at least two images have been processed
        if (dataBuffer.size() > 1) {

            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;
            std::string matcherType = "MAT_BF";                     // MAT_BF, MAT_FLANN
            std::string descriptorTypeForMatching = "DES_BINARY";   // DES_BINARY, DES_HOG
            std::string selectorType = "SEL_KNN";                   // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorTypeForMatching, matcherType, selectorType);

            totalMatchesNum += matches.size();
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            // visualize matches between current and previous image
            if (bVis) {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                std::cout << "Press key to continue to next image" << std::endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }
        // accumulate total time
        tTotal += t1 + t2;
    } // eof loop over all images

    std::cout << std::endl;
    std::cout << "**************SUMMARY*************" << std::endl;
    std::cout << detectorType << " + " << descriptorType << std::endl;
    std::cout << "Total number of Key-points = " << totalKeyPointsNum << std::endl;
    std::cout << "Total number of Matches = " << totalMatchesNum << std::endl;
    std::cout << "Total Time Consumption = " << tTotal * 1000.0 << " ms" << std::endl;
    std::cout << "Ratio = " << totalMatchesNum / (tTotal * 1000.0) << " matches/ms" << std::endl;
    std::cout << "**********************************" << std::endl;

    return 0;
}

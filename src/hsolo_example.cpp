// Copyright 2021 National Technology & Engineering Solutions of Sandia, LLC (NTESS). 
// Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government retains certain rights in this software.
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
    #include <opencv2/features2d.hpp>
#else
    #include <opencv2/xfeatures2d.hpp>
#endif
#include <opencv2/imgcodecs.hpp>
#include "HSolo.h"

using namespace std;
using namespace cv;

int main(int argc, const char * argv[]) {
    
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
    cv::Ptr<Feature2D> sift = SIFT::create(0, 3, 0.04, 10.0, 1.6);
#else
    cv::Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10.0, 1.6);
#endif

    cout << "Loading Images..." << endl;
    Mat img1 = imread("../example_data/vegas1.jpg", IMREAD_GRAYSCALE);
    
    Mat img2 = imread("../example_data/vegas2.jpg", IMREAD_GRAYSCALE);
    
    cout << "Running SIFT on Image 1..." << endl;
    vector<KeyPoint> keypoints1;
    Mat descriptors1;
        
    sift->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    cout << "Found " << keypoints1.size() << " keypoints..." << endl;
    
    cout << "Running SIFT on Image 2..." << endl;
    vector<KeyPoint> keypoints2;
    Mat descriptors2;
    
    sift->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
    cout << "Found " << keypoints2.size() << " keypoints..." << endl;
    
    cout << "Finding Feature Matches..." << endl;
    
    double matchRatio = 0.99;
    
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > matches;
    matcher->knnMatch( descriptors1, descriptors2, matches, 2 );
    
    std::vector<DMatch> candidate_matches;
    vector<HSoloCorrespondence> hsolo_matches;
    
    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i][0].distance < matchRatio * matches[i][1].distance)
        {
            DMatch m = matches[i][0];
            candidate_matches.push_back(m);
            
            HSoloCorrespondence correspondence;
            
            correspondence.pt1_x = keypoints1[m.queryIdx].pt.x;
            correspondence.pt1_y = keypoints1[m.queryIdx].pt.y;
            correspondence.pt1_scale = keypoints1[m.queryIdx].size;
            correspondence.pt1_ori = (keypoints1[m.queryIdx].angle * M_PI) / 180.0;

            correspondence.pt2_x = keypoints2[m.trainIdx].pt.x;
            correspondence.pt2_y = keypoints2[m.trainIdx].pt.y;
            correspondence.pt2_scale = keypoints2[m.trainIdx].size;
            correspondence.pt2_ori = (keypoints2[m.trainIdx].angle * M_PI) / 180.0;
            
            hsolo_matches.push_back(correspondence);
        }
    }
        
    cout << "Found " << hsolo_matches.size() << " candidate matches..." << endl;
    
    cout << "Running OpenCV findHomography..." << endl;
    
    std::vector<Point2f> pts1;
    std::vector<Point2f> pts2;
    
    for( size_t i = 0; i < candidate_matches.size(); i++ )
    {
        pts1.push_back( keypoints1[candidate_matches[i].queryIdx].pt );
        pts2.push_back( keypoints2[candidate_matches[i].trainIdx].pt );
    }
    
    std::vector<char> cvinliers;
    Mat H = findHomography( pts1, pts2, RANSAC, 2.0, cvinliers );
    
    cout << "OpenCV estimated homography:" << endl;
    cout << H << endl;
    
    Mat outImg;
    drawMatches(img1, keypoints1, img2, keypoints2, candidate_matches, outImg, Scalar::all(-1), Scalar::all(-1), cvinliers);
    
    namedWindow("OpenCV findHomography Result");
    imshow("OpenCV findHomography Result", outImg);

    cout << "Running HSolo..." << endl;
    
    HSoloResult hsoloResult;

    try {
        HSolo hsolo;
        HSoloConfiguration config;
        config.setErrorThreshold(2.0);
        config.setRefineSolution(true);

        hsoloResult = hsolo.findHomography(hsolo_matches, config);
            
        cout << "HSolo estimated homography:" << endl;
        cout << hsoloResult.H << endl;
        cout << "Inliers: " << hsoloResult.support << endl;
        cout << "Score: " << hsoloResult.score << endl;
    } catch(const char* msg) {
        cerr << msg << endl;
    }
    
    std::vector<DMatch> final_matches_hsolo;
    
    for(int i;  i < hsoloResult.inliers.size(); i++) {
        if(hsoloResult.inliers[i]) {
            final_matches_hsolo.push_back(candidate_matches[i]);
        }
    }
    
    Mat outImg2;
    drawMatches(img1, keypoints1, img2, keypoints2, final_matches_hsolo, outImg2);
    
    namedWindow("HSolo Result");
    imshow("HSolo Result", outImg2);
    waitKey(0);
    
    destroyWindow("HSolo Result");
    destroyWindow("OpenCV findHomography Result");
    
    return 0;
}


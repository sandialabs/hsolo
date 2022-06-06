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
#pragma once

#include "HSoloTypes.hpp"

#include <cmath>
#include <fstream>
#include <queue>
#include <float.h>

#define SQRT2 1.4142135623730951

struct HSoloCorrespondence;

/**
 * Calculate the number of required RANSAC iterations 
 * 
 * @param inlier_rate estimated inlier rate
 * @param confidence confidence [0.0, 1.0] of finding a correct result
 * @param sample_size minimum number of samples to solve a model
 * @return number of iterations
 */
size_t calcRANSACIters(float inlier_rate, float confidence, int sample_size);

/**
 * Find the indexes of candidate correspondences with reprojection error less than the threshold
 * 
 * @param reprojection_errors reprojection error associated with each candidate correspondence
 * @param error_thresh threshold in pixels that determines if a correspondence is considered an inlier
 * @return indexes of inlier correspondences
 */
std::vector<size_t> findInliers(std::vector<double> &reprojection_errors, double error_thresh);

/**
 * Get the correpondences from a inlier set of indexes
 * 
 * @param correspondences set of candidate correspondences
 * @param inliers list of inlier indicies
 * @return inlier correspondences
 */
std::vector<HSoloCorrespondence> getInlierCorrespondences(std::vector<HSoloCorrespondence> &correspondences, const std::vector<bool>& inliers);

/**
 * get a random sample from the top N correspondences by reprojection error
 * 
 * @param corrs set of candidate correspondences
 * @param H homogrpahy
 * @param samples std::vector store sampled correspondences
 * @param median median value of the top num_samples will be stored in here
 * @param num_samples number of correspondences ot sample
 */
void sampleTopN(std::vector<HSoloCorrespondence> &corrs, const Eigen::Matrix3f &H, 
                std::vector<size_t> &samples, double &median, size_t num_samples);

/**
 * Calculate the reprojection errors, inliers, score, and support 
 * 
 * @param corrs set of candidate correspondences
 * @param H homogrpahy
 * @param errors std::vector to store reprojection errors
 * @param sq_err_thresh squared threshold in pixels that determines if a correspondence is considered an inlier
 * @param inliers std::vector to store inliers
 * @param score reference to double where the score will be stored
 * @return support for H in correpondence set
 */
size_t calcSupport(std::vector<HSoloCorrespondence> &corrs, const Eigen::Matrix3f &H, TransformSolver * solver, 
                   std::vector<double> &errors, double sq_err_thresh, std::vector<bool> &inliers, double &score);

/**
 * Returns true if the triple (x1, y1), (x2, y2), (x3,y3) are co-linear
 */        
bool isTripleCollinear(double x1, double y1, double x2, double y2, double x3, double y3);

/**
Check all combinations of correspondences to ensure no 3 points are collinear
Total of n choose 3 combinations 
**/
bool isSampleValid(std::vector<HSoloCorrespondence> &correspondences);
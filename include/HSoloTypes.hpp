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

#include<Eigen/Dense>

#include "TransformValidator.hpp"

#include <vector>
#include <iostream>

#define SQRT2 1.4142135623730951

/**
    Represents a point corredondence including scales and orientations
*/
struct HSoloCorrespondence
{
    double pt1_x;     // x-coordinate for 1st correspondence
    double pt1_y;     // y-coordinate for 1st correspondence
    double pt2_x;     // x-coordinate for 2nd correspondence
    double pt2_y;     // y-coordinate for 2nd correspondence
    double pt1_ori;   // orientation for 1st correspondence
    double pt1_scale; // scale for 1st correspondence
    double pt2_ori;   // orientation for 2nd correspondence
    double pt2_scale; // scale for 2nd correspondence
};

/**
    Represents the configuration for a given HSolo run
*/
class HSoloConfiguration
{
public:
    double getConfidence() { return confidence; }
    double getErrorThreshold() { return error_threshold; }
    double getFilteredInlierRate() { return filtered_inlier_rate; }
    int getMaxIterations() { return max_iters; }
    double getRunInnerRANSACThresh() { return run_inner_ransac_thresh; }
    int getSearchTopNfiltered() { return search_top_n_filtered; }
    bool getRefineSolution(){ return refine_solution; }

    void addTransformValidator(TransformValidator* validator) {
        transformValidators.push_back(validator);
    }

    void setConfidence(double conf)
    {
        if (conf <= 0.0 || conf > 1.0)
        {
            throw "Confidence value must be (0.0, 1.0]";
        }

        confidence = conf;
    }
    void setErrorThreshold(double thresh)
    {
        if (thresh < 0.0)
        {
            throw "Error Threshold must be > 0.0";
        }

        error_threshold = thresh;
    }
    void setFilteredInlierRate(double fir)
    {
        if (fir <= 0.0 || fir > 1.0)
        {
            throw "Filtered inlier rate value must be (0.0, 1.0]";
        }

        filtered_inlier_rate = fir;
    }
    void setMaxIterations(int miters)
    {
        if (miters <= 0)
        {
            throw "Maximum iterations must be > 0";
        }

        max_iters = miters;
    }
    void setRunInnerRANSACThresh(double rirth)
    {
        if (rirth <= 0)
        {
            throw "Run inner RANSAC threshold must be > 0.0";
        }

        run_inner_ransac_thresh = rirth;
    }
    void setSearchTopNfiltered(int searchTopN)
    {
        if (searchTopN <= 4)
        {
            throw "Search Top N filtered must be >= 4";
        }

        search_top_n_filtered = searchTopN;
    }
    
    void setRefineSolution(bool refine_solution_in)
    {
        refine_solution = refine_solution_in;

    }

    std::vector<TransformValidator*> transformValidators;

private:
    /* Likelihood that a correct solution will be found.
        The higher the confidence, the more matches will be run. */
    double confidence = 0.95;

    /* Maximum reproduction error (in pixels) for a match to be considered an inlier */
    double error_threshold = 4.0;

    /* Estimated inlier rate of matches after being filtered by the initial homography estimate */
    double filtered_inlier_rate = 0.70;

    /* Number of top-N filtered matches to be searched during RANSAC */
    size_t search_top_n_filtered = 20;

    /* Maximum number of iterations allowed to run */
    size_t max_iters = 1000;

    /* Threshld to skip the inner RANSAC step */
    double run_inner_ransac_thresh = 20;

    /* Boolean that determines whether to refine the solution */
    bool refine_solution = true;
};

/**
    Base struct representing the type of transform being solved
 */
struct TransformSolver
{
    virtual int getMinSampleSize() = 0;
    virtual double calcReprojectionError(HSoloCorrespondence &corr, Eigen::Matrix3f H) = 0;
    virtual std::vector<Eigen::Matrix3f> solve(std::vector<HSoloCorrespondence> &corrs) = 0;
};

/**
    Contains the results of a HSolo run
*/
struct HSoloResult
{
    Eigen::Matrix3f H;                       // Estimated homography matrix
    std::vector<bool> inliers;               // Vector of inliers belonging to the estimated homography
    std::vector<double> reprojection_errors; // Reprojection errors associated with each candidate correspondence
    size_t support = 0;                      // Number of inliers that are correctly matched by estimated homography
    double score = 0.0;                      // A special measure of the overall error of an estimated solution. Higher values are better.
    size_t num_iters = 0;                    // Actual number of iterations
    size_t num_inner_ransac_iters = 0;       // Number of inner RANSAC iterations run by the algorithm
};

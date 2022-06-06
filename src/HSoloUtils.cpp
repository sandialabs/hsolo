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

#include "HSoloUtils.h"

#include <iostream>

using namespace std;
using namespace Eigen;

vector<size_t> findInliers(vector<double> &reprojection_errors, double error_thresh)
{
    vector<size_t> inliers;

    for (size_t i = 0; i < reprojection_errors.size(); ++i)
    {
        if (reprojection_errors[i] <= error_thresh)
        {
            inliers.push_back(i);
        }
    }

    return inliers;
}

size_t calcRANSACIters(float inlier_rate, float confidence, int sample_size)
{
    return size_t(log(1.0 - confidence) / log(1.0 - pow(inlier_rate, sample_size)));
}

void sampleTopN(vector<HSoloCorrespondence> &corrs, const Matrix3f &H,
                vector<size_t> &samples, double &median, size_t num_samples)
{
    //use a priority queue to track lowest reproj error idexes.  This makes the proccess
    // n * log (num_samples) vs n * log (n) to sort the whole array
    std::priority_queue<std::pair<double, size_t>, std::vector<std::pair<double, size_t>>,
                        std::less<std::pair<double, size_t>>>
        q;

    double err = 0.0;

    for (size_t i = 0; i < corrs.size(); i++)
    {
        // apply transform
        double t1 = H(0, 0) * corrs[i].pt1_x + H(0, 1) * corrs[i].pt1_y + H(0, 2);
        double t2 = H(1, 0) * corrs[i].pt1_x + H(1, 1) * corrs[i].pt1_y + H(1, 2);
        double t3 = H(2, 0) * corrs[i].pt1_x + H(2, 1) * corrs[i].pt1_y + H(2, 2);

        double errx = corrs[i].pt2_x - (t1 / t3);
        double erry = corrs[i].pt2_y - (t2 / t3);

        err = errx * errx + erry * erry;

        //only add to the queue if this corr is better
        if (q.size() < num_samples)
        {
            q.push(std::pair<double, size_t>(err, i));
        }
        else if (q.top().first > err)
        {
            q.pop();
            q.push(std::pair<double, size_t>(err, i));
        }
    }

    median = 0.0;

    for (size_t i = 0; i < num_samples; ++i)
    {
        samples[i] = q.top().second;

        if (num_samples % 2 == 0 && i == num_samples / 2 - 1)
            median += q.top().first;

        if (i == num_samples / 2)
            median += q.top().first;

        q.pop();
    }

    if (num_samples % 2 == 0)
        median /= 2.0;
}

size_t calcSupport(vector<HSoloCorrespondence> &corrs, const Matrix3f &H, TransformSolver *solver,
                   vector<double> &errors, double sq_err_thresh,
                   vector<bool> &inliers,
                   double &score)
{
    double *err_ptr = reinterpret_cast<double *>(errors.data());
    score = 0;

    size_t support = 0;

    for (size_t i = 0; i < corrs.size(); i++)
    {
        err_ptr[i] = solver->calcReprojectionError(corrs[i], H);

        if (err_ptr[i] <= sq_err_thresh)
        {
            inliers[i] = true;
            //add negative support to allow for maximization of support.  0 = perfect match
            score += 1.0 - err_ptr[i] / sq_err_thresh;
            support++;
        }
        else
        {
            inliers[i] = false;
        }
    }

    return support;
}

vector<HSoloCorrespondence> getInlierCorrespondences(vector<HSoloCorrespondence> &correspondences, const vector<bool> &inliers)
{
    vector<HSoloCorrespondence> inlierCorrs;

    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        if (inliers[i])
            inlierCorrs.push_back(correspondences[i]);
    }

    return inlierCorrs;
}

/**
Returns true if the three points (x1, y1), (x2, y2), and (x3, y3) are collinear
**/
bool isTripleCollinear(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double result = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
    return std::abs(result) <= 1e-9;
}

/**
    Check all combinations of correspondences to ensure no 3 points are collinear
    Total of n choose 3 combinations 
    **/
bool isSampleValid(std::vector<HSoloCorrespondence> &correspondences)
{
    // Since 2 points will always be on a line
    if (correspondences.size() <= 2)
        return true;

    // Else, check all other combinations
    for (int i = 0; i < correspondences.size() - 2; i++)
    {
        for (int j = i + 1; j < correspondences.size() - 1; j++)
        {
            for (int k = j + 1; k < correspondences.size(); k++)
            {
                if (isTripleCollinear(correspondences[i].pt1_x, correspondences[i].pt1_y,
                                      correspondences[j].pt1_x, correspondences[j].pt1_y,
                                      correspondences[k].pt1_x, correspondences[k].pt1_y))
                    return false;
            }
        }
    }
    return true;
}
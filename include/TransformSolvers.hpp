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

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>

struct ProjectiveTransformSolver : TransformSolver
{
    int getMinSampleSize()
    {
        return 4;
    };

    std::vector<Eigen::Matrix3f> solve(std::vector<HSoloCorrespondence> &corrs)
    {
        std::vector<Eigen::Matrix3f> results;

        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;

        for (size_t i = 0; i < corrs.size(); i++)
        {
            src.push_back(cv::Point2d(corrs[i].pt1_x, corrs[i].pt1_y));
            dst.push_back(cv::Point2d(corrs[i].pt2_x, corrs[i].pt2_y));
        }

        cv::Mat H = cv::findHomography(src, dst, 0);

        if (H.rows == 3 && H.cols == 3) {

            Eigen::Matrix3f He = Eigen::Matrix3f::Identity();
            cv::cv2eigen(H, He);
            results.push_back(He);
        }

        return results;
    }

    double calcReprojectionError(HSoloCorrespondence &corr, Eigen::Matrix3f H)
    {
        // default L2 error
        double t1 = H(0, 0) * corr.pt1_x + H(0, 1) * corr.pt1_y + H(0, 2);
        double t2 = H(1, 0) * corr.pt1_x + H(1, 1) * corr.pt1_y + H(1, 2);
        double t3 = H(2, 0) * corr.pt1_x + H(2, 1) * corr.pt1_y + H(2, 2);

        double errx = corr.pt2_x - (t1 / t3);
        double erry = corr.pt2_y - (t2 / t3);

        return errx * errx + erry * erry;
    }
};

struct AffineTransformSolver : ProjectiveTransformSolver
{
    int getMinSampleSize() override
    {
        return 3;
    };

    std::vector<Eigen::Matrix3f> solve(std::vector<HSoloCorrespondence> &corrs) override
    {

        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;

        for (size_t i = 0; i < corrs.size(); i++)
        {
            src.push_back(cv::Point2d(corrs[i].pt1_x, corrs[i].pt1_y));
            dst.push_back(cv::Point2d(corrs[i].pt2_x, corrs[i].pt2_y));
        }

        cv::Mat H = cv::estimateRigidTransform(src, dst, true);

        Eigen::Matrix3f He = Eigen::Matrix3f::Identity();

        He(0, 0) = H.at<double>(0, 0);
        He(0, 1) = H.at<double>(0, 1);
        He(0, 2) = H.at<double>(0, 2);
        He(1, 0) = H.at<double>(1, 0);
        He(1, 1) = H.at<double>(1, 1);
        He(1, 2) = H.at<double>(1, 2);

        std::vector<Eigen::Matrix3f> results;
        results.push_back(He);

        return results;
    }
};

struct SimilarityTransformSolver : ProjectiveTransformSolver
{
    int getMinSampleSize() override
    {
        return 2;
    };

    std::vector<Eigen::Matrix3f> solve(std::vector<HSoloCorrespondence> &corrs) override
    {

        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;

        for (size_t i = 0; i < corrs.size(); i++)
        {
            src.push_back(cv::Point2d(corrs[i].pt1_x, corrs[i].pt1_y));
            dst.push_back(cv::Point2d(corrs[i].pt2_x, corrs[i].pt2_y));
        }

        std::vector<uchar> inliers;
        cv::Mat H = cv::estimateRigidTransform(src, dst, false);

        Eigen::Matrix3f He = Eigen::Matrix3f::Identity();

        He(0, 0) = H.at<double>(0, 0);
        He(0, 1) = H.at<double>(0, 1);
        He(0, 2) = H.at<double>(0, 2);
        He(1, 0) = H.at<double>(1, 0);
        He(1, 1) = H.at<double>(1, 1);
        He(1, 2) = H.at<double>(1, 2);

        std::vector<Eigen::Matrix3f> results;
        results.push_back(He);

        return results;
    }
};

struct TranslationTransformSolver : ProjectiveTransformSolver
{
    std::vector<Eigen::Matrix3f> solve (std::vector<HSoloCorrespondence> &corrs) override
    {
        double mn_xt = 0.0;
        double mn_yt = 0.0;

        for (size_t i = 0; i < corrs.size(); ++i)
        {
            mn_xt += corrs[i].pt2_x - corrs[i].pt1_x;
            mn_yt += corrs[i].pt2_y - corrs[i].pt1_y;
        }

        mn_xt /= corrs.size();
        mn_yt /= corrs.size();

        Eigen::Matrix3f H;

        H(0, 0) = 1.0;
        H(0, 1) = 0.0;
        H(0, 2) = mn_xt;
        H(1, 0) = 0.0;
        H(1, 1) = 1.0;
        H(1, 2) = mn_yt;
        H(2, 0) = 0.0;
        H(2, 1) = 0.0;
        H(2, 2) = 1.0;

        return std::vector<Eigen::Matrix3f>{H};
    }

    std::vector<Eigen::Matrix3f> solveNormalized (std::vector<HSoloCorrespondence> &corrs) {
        return solve(corrs);
    }

    int getMinSampleSize() override
    {
        return 1;
    }
};

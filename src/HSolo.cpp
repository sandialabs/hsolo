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

#include "HSolo.h"
#include "HSoloUtils.h"
#include "TransformValidator.hpp"

#include <iostream>
#include <random>
#include <algorithm>
#include <set>

using namespace std;
using namespace Eigen;

random_device device;
mt19937 generator(device());
uniform_real_distribution<double> unif(1e-7, 0.02);
const double DEFAULT_LOWER_DET_LIMIT = 0.1f;

bool HSolo::validateResults(HSoloResult& result, HSoloConfiguration &config)
{
    return validateTransform(result.H, config) &&
           result.support >= solver->getMinSampleSize();
}

bool HSolo::validateTransform(Eigen::Matrix3f trans, HSoloConfiguration &config) {
    for (int i = 0; i < config.transformValidators.size(); i++) {
        if( !config.transformValidators[i]->isTransformValid(trans)) {
            return false;
        }
    }

    return true;
}

void HSolo::refineHomography(vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config,
                             HSoloResult &result,
                             size_t max_iters)
{
    bool got_better = true;
    size_t iter = 0;

    vector<double> errors(correspondences.size(), 0.0);
    vector<bool> uinliers(correspondences.size(), false);

    size_t support = 0;
    double score = 0.0;

    double sq_error_threshold = config.getErrorThreshold() * config.getErrorThreshold();

    while (got_better && iter < max_iters)
    {

        vector<HSoloCorrespondence> inlier_corrs = getInlierCorrespondences(correspondences, result.inliers);

        std::vector<Eigen::Matrix3f> H_candidates = solver->solve(inlier_corrs);

        for(int i = 0; i < H_candidates.size(); i++) 
        {
            Eigen::Matrix3f H_candidate = H_candidates[i];
            
            score = 0.0;
            support = calcSupport(correspondences, H_candidate, solver, errors, sq_error_threshold, uinliers, score);

            got_better = score > result.score && validateTransform(H_candidate, config);

            if (got_better)
            {
                result.inliers = uinliers;
                result.support = support;
                result.score = score;
                result.support = support;
                result.H = H_candidate;
            }
        }

        iter++;
    }
}

Matrix3f HSolo::localSearchIntialHomography(vector<HSoloCorrespondence> &correpondences, 
                                            HSoloConfiguration &config, Matrix3f &Hinit,
                                            size_t &num_inner_ransac_runs, size_t num_iters)
{
    vector<size_t> sample_idxs(config.getSearchTopNfiltered());

    //get the search_top_n closest correspondences by reprojection error
    double median_err = 0.0;
    sampleTopN(correpondences, Hinit, sample_idxs, median_err, config.getSearchTopNfiltered());

    //only run inner ransac if the median reprojection error is low enough
    if (median_err > config.getRunInnerRANSACThresh() * config.getRunInnerRANSACThresh() )
        return Hinit;

    size_t best_support = 0;
    double score = 0.0;

    Matrix3f best_H;
    vector<double> errors(correpondences.size(), 0.0);
    vector<bool> inliers(correpondences.size(), false);

    num_inner_ransac_runs++;

    double sq_error_threshold = config.getErrorThreshold() * config.getErrorThreshold();

    for (size_t i = 0; i < num_iters; i++)
    {
        //randomly sample 4 indexes from the sample_idxs list to use in solution
        vector<size_t> mysample_idxs;
        std::sample(sample_idxs.begin(), sample_idxs.end(), std::back_inserter(mysample_idxs),
               solver->getMinSampleSize(), generator);

        vector<HSoloCorrespondence> mysample(mysample_idxs.size());

        for (size_t j = 0; j < mysample_idxs.size(); j++)
        {
            mysample[j] = correpondences[mysample_idxs[j]];
        }

        if (isSampleValid(mysample))
        {

            std::vector<Eigen::Matrix3f> H_candidates = solver->solve(mysample);
            for(int i = 0; i < H_candidates.size(); i++) 
            {
                Eigen::Matrix3f H_candidate = H_candidates[i];
                score = 0.0;
                size_t support = calcSupport(correpondences, H_candidate, solver, errors, sq_error_threshold,
                                            inliers, score);

                if (support >= best_support && validateTransform(H_candidate, config))
                {
                    best_support = support;
                    best_H = H_candidate;
                }
            }

        }
    }

    return best_H;
}

/**
 * Generate an approx of the affine homography between the images using the appromiations of the
 * local transforms for each point. 
 */
Matrix3f HSolo::findHInit(HSoloCorrespondence &correpondence)
{
    double x1 = correpondence.pt1_x;
    double y1 = correpondence.pt1_y;

    double x2 = correpondence.pt2_x;
    double y2 = correpondence.pt2_y;

    //TODO:  Directly build inverse of P using negative translations / rotation + 1/scale
    Matrix3f P = Matrix3f::Identity();
    Matrix3f P2 = Matrix3f::Identity();

    P(0, 0) = cos(correpondence.pt1_ori);
    P(0, 1) = -sin(correpondence.pt1_ori);
    P(1, 0) = sin(correpondence.pt1_ori);
    P(1, 1) = cos(correpondence.pt1_ori);

    P2(0, 0) = correpondence.pt1_scale;
    P2(1, 1) = correpondence.pt1_scale;

    P = P2 * P;
    P(0, 2) = x1;
    P(1, 2) = y1;

    Matrix3f Q = Matrix3f::Identity();
    Matrix3f Q2 = Matrix3f::Identity();

    Q(0, 0) = cos(correpondence.pt2_ori);
    Q(0, 1) = -sin(correpondence.pt2_ori);
    Q(1, 0) = sin(correpondence.pt2_ori);
    Q(1, 1) = cos(correpondence.pt2_ori);

    Q2(0, 0) = correpondence.pt2_scale;
    Q2(1, 1) = correpondence.pt2_scale;

    Q = Q2 * Q;
    Q(0, 2) = x2;
    Q(1, 2) = y2;

    Matrix3f Hi = Q * P.inverse();

    return Hi;
}

HSoloResult HSolo::findHomography(vector<HSoloCorrespondence> &correpondences, HSoloConfiguration &config)
{
    DeterminantTransformValidator* detValidator = new DeterminantTransformValidator();
    detValidator->lowerLimit = DEFAULT_LOWER_DET_LIMIT;
    config.addTransformValidator(detValidator);

    solver = new ProjectiveTransformSolver();
    return findTransform(correpondences, config);
}

HSoloResult HSolo::findAffine(vector<HSoloCorrespondence> &correpondences, HSoloConfiguration &config)
{
    DeterminantTransformValidator* detValidator = new DeterminantTransformValidator();
    detValidator->lowerLimit = DEFAULT_LOWER_DET_LIMIT;
    config.addTransformValidator(detValidator);

    solver = new AffineTransformSolver();
    return findTransform(correpondences, config);
}

HSoloResult HSolo::findSimilarity(vector<HSoloCorrespondence> &correpondences, HSoloConfiguration &config)
{
    DeterminantTransformValidator* detValidator = new DeterminantTransformValidator();
    detValidator->lowerLimit = DEFAULT_LOWER_DET_LIMIT;
    config.addTransformValidator(detValidator);

    solver = new SimilarityTransformSolver();
    return findTransform(correpondences, config);
}

HSoloResult HSolo::findTranslation(vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config)
{
    DeterminantTransformValidator* detValidator = new DeterminantTransformValidator();
    detValidator->lowerLimit = DEFAULT_LOWER_DET_LIMIT;
    config.addTransformValidator(detValidator);
    
    solver = new TranslationTransformSolver();
    return findTransform(correspondences, config);
}


HSoloResult HSolo::findTransform(vector<HSoloCorrespondence> &correpondences, HSoloConfiguration &config)
{
    if (!solver)
    {
        throw "Transform solver is not not set!";
    }

    // Need enough correspondences to solve a model
    if (correpondences.size() < solver->getMinSampleSize())
    {
        throw "The number of correspondences must be >= the minimum number to solve for a transform, " +
            to_string(solver->getMinSampleSize());
    }

    // Need enough correspondences to solve a model
    if (correpondences.size() < config.getSearchTopNfiltered())
    {
        throw "The number of correspondences must be >= the value of searchTopNFiltered, " +
            to_string(config.getSearchTopNfiltered());
    }

    size_t run_iters = config.getMaxIterations();

    //calculate number of iterations to run for local optimization
    size_t inner_iters = calcRANSACIters(config.getFilteredInlierRate(), config.getConfidence(), solver->getMinSampleSize());

    //produce random order for iteration
    vector<size_t> sample_order;

    for (size_t i = 0; i < correpondences.size(); ++i)
    {
        sample_order.push_back(i);
    }

    shuffle(sample_order.begin(), sample_order.end(), generator);

    HSoloResult result;
    result.support = 0;
    result.inliers.resize(correpondences.size(), false);
    result.reprojection_errors.resize(correpondences.size(), 0.0);

    vector<double> errors(correpondences.size(), 0.0);
    vector<bool> inliers(correpondences.size(), false);

    double sq_error_threshold = config.getErrorThreshold() * config.getErrorThreshold();

    size_t num_inner_ransac_runs = 0;
    size_t num_iters_run = 0;

    while (num_iters_run < run_iters && num_iters_run < sample_order.size())
    {
        //get initial H estimate using single correpondence
        Matrix3f Hinit = findHInit(correpondences[sample_order[num_iters_run]]);

        if (validateTransform(Hinit, config))
        {

            //perform local search for improved H
            Matrix3f H_candidate = localSearchIntialHomography(correpondences, config, Hinit,
                                                               num_inner_ransac_runs, inner_iters);

            //calcuate support for the local search result

            double score = 0.0;
            size_t support = calcSupport(correpondences, H_candidate, solver,
                                         errors, sq_error_threshold,
                                         inliers, score);

            bool didImprove = (support > result.support) || (support == result.support && score > result.score);

            if (didImprove && validateTransform(H_candidate, config))
            {
                result.inliers = inliers;
                result.H = H_candidate;
                result.support = support;
                result.reprojection_errors = errors;
                result.score = score;
                run_iters = min((size_t)config.getMaxIterations(),
                                calcRANSACIters((double)support / correpondences.size(),
                                                config.getConfidence(), 1));
            }
        }

        num_iters_run++;
    }

    if (!validateResults(result, config))
    {
        throw "HSolo failed to find a solution.";
    }

    if (config.getRefineSolution())
    {
        refineHomography(correpondences, config, result);
    }

    result.num_iters = num_iters_run;
    result.num_inner_ransac_iters = num_inner_ransac_runs;
    return result;
}

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
#include "TransformSolvers.hpp"

class HSolo {

    public:

        /* Solver object (for calculating transformations) */
        TransformSolver * solver = NULL;

        /**
         * Find a linear transfomr from a set of candidate correspondences based on the TrasnformSolver
         * 
         * @param correspondences Set of candidate correspondences
         * @param config configuration 
         * @return result containg the solved H and inliers
         */
        HSoloResult findTransform(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config);

        /**
         * Find a 9-DOF Homography from a set of candidate correspondences.
         * 
         * @param correspondences Set of candidate correspondences
         * @param config configuration 
         * @return result containg the solved H and inliers
         */
        HSoloResult findHomography(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config);

         /**
         * Estimate a 6-DOF Affine Transformation from a set of candidate correspondences.
         * 
         * @param correspondences Set of candidate correspondences
         * @param config configuration 
         * @return result containg the solved H and inliers
         */
        HSoloResult findAffine(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config);

        /**
         * Estimate a 3-DOF (translation, rotation, scale) Similarity Transformation from a set of candidate correspondences.
         * 
         * @param correspondences Set of candidate correspondences
         * @param config configuration 
         * @return result containg the solved H and inliers
         */
        HSoloResult findSimilarity(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config);

        HSoloResult findTranslation(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config);
    
        /**
         * Validate the HSolo result returned by findTransform()
         * 
         * @param HSolo result object
         * @return boolean corresponding to whether the result is valid or not
         */
        bool validateResults(HSoloResult& result, HSoloConfiguration &config);

        bool validateTransform(Eigen::Matrix3f transf, HSoloConfiguration &config);
        
    private:
       
        /**
         * Refine a homography by iterating and resolving with entire inlier set
         * 
         * @param correspondences Set of inlier correspondences
         * @param result result object containing the estimated homography and inliers
         * @param sq_error_thresh squared threshold in pixels that determines if a correspondence is considered an inlier
         * @param max_iters the maximum number of iterations allowed to run.  default = 25
         */
        void refineHomography(std::vector<HSoloCorrespondence> &correspondences, HSoloConfiguration &config, HSoloResult &result,
                                size_t max_iters = 25);

        /**
         * Create a filtered correspondence set and use 4point RANSAC to search it for for a homography
         * 
         * @param correspondences Set of candidate correspondences
         * @param config configuration 
         * @param Hinit initial estimate of the homography
         * @param num_iters number of RANSAC iterations to run. default = 10
         * @return estimated homography
         */
        Eigen::Matrix3f localSearchIntialHomography(std::vector<HSoloCorrespondence> &correspondences, 
                                HSoloConfiguration &config,
                                Eigen::Matrix3f &Hinit, size_t &num_inner_ransac_runs, size_t num_iters = 10);

        Eigen::Matrix3f findHInit(HSoloCorrespondence &correpondence);
};

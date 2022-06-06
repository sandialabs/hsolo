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

#include "mex.hpp"
#include "mexAdapter.hpp"
#include "HSolo.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

// Matlab wrapper for the HSolo C++ source code.

class MexFunction : public matlab::mex::Function {

    // Define private class variables
    ArrayFactory factory;
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr;

public:

    MexFunction() { // Constructor
        matlabPtr = getEngine();
    }

    void operator()(ArgumentList outputs, ArgumentList inputs) {
        checkArguments(outputs, inputs);

        HSolo hsolo; // Create HSolo object
        HSoloConfiguration config;

        try { // Set HSolo parameters
            config.setConfidence(inputs[7][0]);
            config.setMaxIterations(inputs[8][0]);
            config.setSearchTopNfiltered(inputs[9][0]);
            config.setRunInnerRANSACThresh(inputs[10][0]);
            config.setFilteredInlierRate(inputs[11][0]);
            config.setErrorThreshold(inputs[12][0]);
            config.setRefineSolution(inputs[13][0]);
        } 
        // Bubble up C++ errors to Matlab
        catch (const char* msg) {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar(msg) })
            );
        }

        // Create vector of correspondences
        std::vector<HSoloCorrespondence> corrs;

        // Get keypoint location, scale, and orientation
        const TypedArray<double> x1y1 = inputs[0];
        const TypedArray<double> x2y2 = inputs[1];
        const TypedArray<double> scales1 = inputs[2];
        const TypedArray<double> scales2 = inputs[3];
        const TypedArray<double> oris1 = inputs[4];
        const TypedArray<double> oris2 = inputs[5];

        // Populate correspondence vector 
        ArrayDimensions dims = inputs[0].getDimensions();
        for (size_t ii = 0; ii < dims[0]; ii++)
        {
            HSoloCorrespondence corr;
            corr.pt1_x = x1y1[ii][0];
            corr.pt1_y = x1y1[ii][1];
            corr.pt2_x = x2y2[ii][0];
            corr.pt2_y = x2y2[ii][1];

            corr.pt1_scale = scales1[ii];
            corr.pt2_scale = scales2[ii];
            corr.pt1_ori = oris1[ii];
            corr.pt2_ori = oris2[ii];

            corrs.push_back(corr);
        }
    
        // Get transform type (similarity=2, affine=3, projective=4)
        TypedArray<double> transformType = inputs[6];

        // Find homography matrix
        HSoloResult result; 
        try {
            if (transformType[0] == 2) { // Similarity
                result = hsolo.findSimilarity(corrs, config);
            }
            else if (transformType[0] == 3) { // Affine
                result = hsolo.findAffine(corrs, config);
            }
            else if (transformType[0] == 4) { // Projective 
                result = hsolo.findHomography(corrs, config);
            }else if(transformType[0] == 1) { // Translation
                result = hsolo.findTranslation(corrs,config);
            }
        }
        // Bubble up C++ errors to Matlab
        catch (const char* msg) {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar(msg) })
            );
        }

        // Copy homography matrix to function output
        TypedArray<float> outputH = factory.createArray<float>({3, 3});
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                outputH[i][j] = result.H(i, j);
            }
        }
        outputs[0] = outputH;

        // Copy list of inliers to function output
        TypedArray<float> outputInliers = factory.createArray<float>({1, result.inliers.size()});
        for(size_t i = 0; i < result.inliers.size(); i++) {
            outputInliers[i] = result.inliers[i];
        }
        outputs[1] = outputInliers; 
    }

    // Validate input arguments
    void checkArguments(ArgumentList outputs, ArgumentList inputs) {

        // Check array types (must be DOUBLE)
        for (size_t i = 0; i < 6; i++) {
            if (inputs[i].getType() != ArrayType::DOUBLE) {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Input must be of type DOUBLE") })
                );
            }
        }

        // Check number of elements in each array
        size_t N = inputs[0].getDimensions()[0];
        for (size_t i = 0; i < 6; i++) {
            if (inputs[i].getDimensions()[0] != N) {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Input arrays must have same number of elements") })
                );
            }
        }
        
        // Check number of columns in each array
        for (size_t i = 0; i < 6; i++) {
            if (i < 2 && inputs[i].getDimensions()[1] != 2) {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Input must be a 2D array") })
                );
            }
            if (i >= 2 && inputs[i].getDimensions()[1] != 1) {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Input must be a 1D array") })
                );
            }
        }

        // Check number of outputs
        if (outputs.size() != 2) {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Two outputs must be returned: (homography matrix, inliers)") })
            );
        }
    }
};


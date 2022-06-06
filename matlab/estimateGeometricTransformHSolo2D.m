% Copyright 2021 National Technology & Engineering Solutions of Sandia, LLC (NTESS). 
% Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government retains certain rights in this software.
%
%    Licensed under the Apache License, Version 2.0 (the "License");
%    you may not use this file except in compliance with the License.
%    You may obtain a copy of the License at
%
%        http://www.apache.org/licenses/LICENSE-2.0
%
%    Unless required by applicable law or agreed to in writing, software
%    distributed under the License is distributed on an "AS IS" BASIS,
%    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%    See the License for the specific language governing permissions and
%    limitations under the License.


% Estimate 2-D geometric transformation from matching point pairs using the 
% HSolo algorithm.
% 
% tform = estimateGeometricTransform2D(matchedPoints1, matchedPoints2, transformType) 
% 
% Returns a 2-D geometric transformation which maps the
% inliers in matchedPoints1 to the inliers in matchedPoints2.
% matchedPoints1 and matchedPoints2 can be M-by-2 matrices of [x,y]
% coordinates or objects of any of the point feature types. 
% transformType can be 'similarity', 'affine' or 'projective'. 

function [tform, inlierIdx,status] = estimateGeometricTransformHSolo2D(matchedPoints1, matchedPoints2, transformType, varargin)
    status = 0;

    % Add optional input parameters
    p = inputParser;
    addOptional(p,'Confidence',95);
    addOptional(p,'MaxNumTrials',1000);
    addOptional(p,'SearchTopN',20);
    addOptional(p,'RunInnerRANSACThresh',25);
    addOptional(p,'FilteredInlierRate', 0.7);
    addOptional(p,'MaxDistance', 2.0);
    addOptional(p,'RefineSolution', true);
    
    % Parse parameters
    parse(p, varargin{:});

    p.Results
    
    % Check input types
    if(isa(matchedPoints1,'cornerPoints') || isa(matchedPoints1,'MSERRegions') ...
        || isa(matchedPoints1, 'numeric'))
        error("HSolo does not currently support type " + class(matchedPoints1))           
    end
    
    % Determine transformType
    switch(transformType)
        case "similarity"
            numMatchedPoints = 2;
        case "affine"
            numMatchedPoints = 3;
        case "projective"
            numMatchedPoints = 4;
        case "translation"
            numMatchedPoints = 1;
        case "rigid"
            error("HSolo does not currently support 'rigid' transforms.")
        otherwise
            error("transformType must be 'translation', 'similarity', 'affine', or 'projective'.")
    end
    
    % Determine if there is enough matched points
    if(length(matchedPoints1) < numMatchedPoints)
        status = 1;
        error(status + ": Matched Points do not contain enough points.");
    end
    
    try
        % Find homography (internal HSolo MEX wrapper)
        [homographyMatrix, inlierIdx] = findHomographyHSoloMEX(...
            double(matchedPoints1.Location), double(matchedPoints2.Location), ...
            double(matchedPoints1.Scale), double(matchedPoints2.Scale), ...
            double(matchedPoints1.Orientation), double(matchedPoints2.Orientation), ...
            numMatchedPoints, ...
            p.Results.Confidence/100, ...
            p.Results.MaxNumTrials, ...
            p.Results.SearchTopN, ...
            p.Results.RunInnerRANSACThresh, ...
            p.Results.FilteredInlierRate, ...
            p.Results.MaxDistance, ...
            p.Results.RefineSolution);
        disp(size(inlierIdx));
    catch exception
        status = 2; 
        error(status + ": "+exception.message);
    end
    
    % Output variables
    tform = projective2d(homographyMatrix);
    inlierIdx = logical(inlierIdx');
    
end



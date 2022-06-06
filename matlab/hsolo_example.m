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

%% Read input image pair
img1 = rgb2gray(imread('../example_data/vegas1.jpg'));
img2 = rgb2gray(imread('../example_data/vegas2.jpg'));

%% Define constants
mthresh = 0.1;
transformType = 'projective'; % similarity, affine, or projective

%% Find matched SURF points
surf1 = detectSURFFeatures(img1, 'MetricThreshold', mthresh);
surf2 = detectSURFFeatures(img2, 'MetricThreshold', mthresh);

[features1, surf1] = extractFeatures(img1, surf1);
[features2, surf2] = extractFeatures(img2, surf2);

matches = matchFeatures(features1, features2, 'MatchThreshold', 10, 'MaxRatio', 0.8, 'Method', 'Approximate');

matchedPoints1 = surf1(matches(:, 1));
matchedPoints2 = surf2(matches(:, 2));

%% Find inliers and estimate homography matrix (Matlab)
subplot(2,1,1); 
try
    tic
    [homographyMatrix, inlierIdx] = estimateGeometricTransform2D(...
        matchedPoints1, ...
        matchedPoints2, ...
        transformType...
    );
    toc
catch ME
    warning("Matlab warning:")
    disp(ME)
end
showMatchedFeatures(img1,img2,matchedPoints1(inlierIdx),matchedPoints2(inlierIdx), 'montage');
legend('vegas1.jpeg', 'vegas2.jpeg')
title('Matched inlier points (Matlab)')

%% Find inliers and estimate homography matrix (HSolo)
subplot(2,1,2); 
tic
[homographyMatrixHSolo, inlierIdxHSolo,status] = estimateGeometricTransformHSolo2D(...
    matchedPoints1, ...
    matchedPoints2, ...
    transformType...
);
toc
showMatchedFeatures(img1,img2,matchedPoints1(inlierIdxHSolo),matchedPoints2(inlierIdxHSolo), 'montage');
legend('vegas1.jpeg', 'vegas2.jpeg')
title('Matched inlier points (HSolo)')

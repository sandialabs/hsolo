// Copyright 2021 National Technology & Engineering Solutions of Sandia, LLC (NTESS). 
// Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government retains certain rights in this software.
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at

//        http://www.apache.org/licenses/LICENSE-2.0

//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "HSolo.h"
#include <stdexcept>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <string>

namespace py = pybind11;

using namespace std;

std::vector<HSoloCorrespondence> prepareData(
    py::array_t<double> x1y1_,
    py::array_t<double> x2y2_,
    py::array_t<double> scales1_,
    py::array_t<double> scales2_,
    py::array_t<double> oris1_,
    py::array_t<double> oris2_,
    int min_sample_size)
{
    py::buffer_info x1y1_buffer = x1y1_.request();
    size_t x1y1_rows = x1y1_buffer.shape[0];
    size_t x1y1_cols = x1y1_buffer.shape[1];

    std::string minSampleSize = to_string(min_sample_size);

    if (x1y1_cols != 2)
    {
        throw std::invalid_argument("x1y1 should be an array with dims [n,2], n>=" + minSampleSize);
    }
    if (x1y1_rows < min_sample_size)
    {
        throw std::invalid_argument("x1y1 should be an array with dims [n,2], n>=" + minSampleSize);
    }

    py::buffer_info x2y2_buffer = x2y2_.request();
    size_t x2y2_rows = x2y2_buffer.shape[0];
    size_t x2y2_cols = x2y2_buffer.shape[1];

    if (x2y2_cols != 2)
    {
        throw std::invalid_argument("x2y2 should be an array with dims [n,2], n>=" + minSampleSize);
    }
    if (x2y2_rows != x1y1_rows)
    {
        throw std::invalid_argument("x1y1 and x2y2 should be the same size");
    }

    py::buffer_info scales1_buffer = scales1_.request();
    size_t scales1_rows = scales1_buffer.shape[0];
    size_t scales1_cols = scales1_buffer.shape[1];

    if (scales1_cols != 1)
    {
        throw std::invalid_argument("scales1 should be an array with dims [n,1], n>=" + minSampleSize);
    }
    if (scales1_rows != x1y1_rows)
    {
        throw std::invalid_argument("x1y1 and scales1 should be the same size");
    }

    py::buffer_info scales2_buffer = scales2_.request();
    size_t scales2_rows = scales2_buffer.shape[0];
    size_t scales2_cols = scales2_buffer.shape[1];

    if (scales2_cols != 1)
    {
        throw std::invalid_argument("scales2 should be an array with dims [n,1], n>=" + minSampleSize);
    }
    if (scales2_rows != x1y1_rows)
    {
        throw std::invalid_argument("x2y2 and scales2 should be the same size");
    }

    py::buffer_info oris1_buffer = oris1_.request();
    size_t oris1_rows = oris1_buffer.shape[0];
    size_t oris1_cols = oris1_buffer.shape[1];

    if (oris1_cols != 1)
    {
        throw std::invalid_argument("oris1 should be an array with dims [n,1], n>=" + minSampleSize);
    }
    if (oris1_rows != x1y1_rows)
    {
        throw std::invalid_argument("x1y1 and oris1 should be the same size");
    }

    py::buffer_info oris2_buffer = oris2_.request();
    size_t oris2_rows = oris2_buffer.shape[0];
    size_t oris2_cols = oris2_buffer.shape[1];

    if (oris2_cols != 1)
    {
        throw std::invalid_argument("oris2 should be an array with dims [n,1], n>=" + minSampleSize);
    }
    if (oris2_rows != x1y1_rows)
    {
        throw std::invalid_argument("x2y2 and oris2 should be the same size");
    }

    double *x1y1_ptr = (double *)x1y1_buffer.ptr;
    std::vector<double> x1y1;
    x1y1.assign(x1y1_ptr, x1y1_ptr + x1y1_buffer.size);

    double *x2y2_ptr = (double *)x2y2_buffer.ptr;
    std::vector<double> x2y2;
    x2y2.assign(x2y2_ptr, x2y2_ptr + x2y2_buffer.size);

    double *scales1_ptr = (double *)scales1_buffer.ptr;
    std::vector<double> scales1;
    scales1.assign(scales1_ptr, scales1_ptr + scales1_buffer.size);

    double *scales2_ptr = (double *)scales2_buffer.ptr;
    std::vector<double> scales2;
    scales2.assign(scales2_ptr, scales2_ptr + scales2_buffer.size);

    double *oris1_ptr = (double *)oris1_buffer.ptr;
    std::vector<double> oris1;
    oris1.assign(oris1_ptr, oris1_ptr + oris1_buffer.size);

    double *oris2_ptr = (double *)oris2_buffer.ptr;
    std::vector<double> oris2;
    oris2.assign(oris2_ptr, oris2_ptr + oris2_buffer.size);


    std::vector<HSoloCorrespondence> corrs;

    for (size_t ii = 0; ii < x1y1_rows; ii++)
    {
        HSoloCorrespondence corr;
        corr.pt1_x = x1y1[ii * 2];
        corr.pt1_y = x1y1[ii * 2 + 1];
        corr.pt2_x = x2y2[ii * 2];
        corr.pt2_y = x2y2[ii * 2 + 1];

        corr.pt1_scale = scales1[ii];
        corr.pt2_scale = scales2[ii];
        corr.pt1_ori = oris1[ii];
        corr.pt2_ori = oris2[ii];

        corrs.push_back(corr);
    }

    return corrs;
}

py::tuple prepareResult(HSoloResult& result, bool get_stats) {
    // no homography found, return None
    if (result.support == 0)
    {
        if (get_stats == true)
        {
            return py::make_tuple(py::none(), py::none(), py::none());
        }
        else
        {
            return py::make_tuple(py::none(), py::none());
        }
    }

    py::array_t<bool> inliers_ = py::array_t<bool>(result.inliers.size());
    py::buffer_info buf3 = inliers_.request();
    bool *ptr3 = (bool *)buf3.ptr;
    for (size_t i = 0; i < result.inliers.size(); i++)
        ptr3[i] = result.inliers[i];

    std::vector<float> H(9);
    H[0] = result.H(0, 0);
    H[1] = result.H(0, 1);
    H[2] = result.H(0, 2);
    H[3] = result.H(1, 0);
    H[4] = result.H(1, 1);
    H[5] = result.H(1, 2);
    H[6] = result.H(2, 0);
    H[7] = result.H(2, 1);
    H[8] = result.H(2, 2);

    py::array_t<float> H_ = py::array_t<float>({3, 3});
    py::buffer_info buf2 = H_.request();
    float *ptr2 = (float *)buf2.ptr;
    for (size_t i = 0; i < 9; i++)
        ptr2[i] = H[i];

    py::dict result_stats;
    result_stats["support"] = result.support;
    result_stats["score"] = result.score;
    result_stats["num_iters"] = result.num_iters;
    result_stats["num_inner_ransac_iters"] = result.num_inner_ransac_iters;

    if (get_stats == true)
    {
        return py::make_tuple(H_, inliers_, result_stats);
    }
    else
    {
        return py::make_tuple(H_, inliers_);
    }
}

py::tuple findTranslation_(py::array_t<double> x1y1_,
                      py::array_t<double> x2y2_,
                      py::array_t<double> scales1_,
                      py::array_t<double> scales2_,
                      py::array_t<double> oris1_,
                      py::array_t<double> oris2_,
                      double threshold,
                      double filtered_inlier_rate,
                      int search_top_n_filtered,
                      double conf,
                      int max_iters,
                      double run_inner_ransac_thresh,
                      double lo_det_limit,
                      double hi_det_limit,
                      bool get_stats,
                      bool refine_solution)
{
    try
    {
        HSoloConfiguration config;

        config.setErrorThreshold(threshold);
        config.setConfidence(conf);
        config.setFilteredInlierRate(filtered_inlier_rate);
        config.setRunInnerRANSACThresh(run_inner_ransac_thresh);
        config.setMaxIterations(max_iters);
        config.setSearchTopNfiltered(search_top_n_filtered);


        std::vector<HSoloCorrespondence> corrs = prepareData(x1y1_, x2y2_, 
                                        scales1_, scales2_,oris1_, oris2_, 3);

        HSolo hsolo;
        HSoloResult result = hsolo.findTranslation(corrs, config);

        return prepareResult(result, get_stats);
    }
    catch (const char *msg)
    {
        throw std::invalid_argument(msg);
    }
}


py::tuple findSimilairty_(py::array_t<double> x1y1_,
                          py::array_t<double> x2y2_,
                          py::array_t<double> scales1_,
                          py::array_t<double> scales2_,
                          py::array_t<double> oris1_,
                          py::array_t<double> oris2_,
                          double threshold,
                          double filtered_inlier_rate,
                          int search_top_n_filtered,
                          double conf,
                          int max_iters,
                          double run_inner_ransac_thresh,
                          bool get_stats,
                          bool refine_solution)
{
    try
    {

        HSoloConfiguration config;

        config.setErrorThreshold(threshold);
        config.setConfidence(conf);
        config.setFilteredInlierRate(filtered_inlier_rate);
        config.setRunInnerRANSACThresh(run_inner_ransac_thresh);
        config.setMaxIterations(max_iters);
        config.setSearchTopNfiltered(search_top_n_filtered);
        config.setRefineSolution(refine_solution); 

        std::vector<HSoloCorrespondence> corrs = prepareData(x1y1_, x2y2_, 
                                        scales1_, scales2_,oris1_, oris2_, 2);

        HSolo hsolo;
        HSoloResult result = hsolo.findSimilarity(corrs, config);

        return prepareResult(result, get_stats);
    }
    catch (const char *msg)
    {
        throw std::invalid_argument(msg);
    }
}

py::tuple findAffine_(py::array_t<double> x1y1_,
                      py::array_t<double> x2y2_,
                      py::array_t<double> scales1_,
                      py::array_t<double> scales2_,
                      py::array_t<double> oris1_,
                      py::array_t<double> oris2_,
                      double threshold,
                      double filtered_inlier_rate,
                      int search_top_n_filtered,
                      double conf,
                      int max_iters,
                      double run_inner_ransac_thresh,
                      bool get_stats,
                      bool refine_solution)
{
    try
    {
        HSoloConfiguration config;
        config.setErrorThreshold(threshold);
        config.setConfidence(conf);
        config.setFilteredInlierRate(filtered_inlier_rate);
        config.setRunInnerRANSACThresh(run_inner_ransac_thresh);
        config.setMaxIterations(max_iters);
        config.setSearchTopNfiltered(search_top_n_filtered);
        config.setRefineSolution(refine_solution);

        std::vector<HSoloCorrespondence> corrs = prepareData(x1y1_, x2y2_, 
                                        scales1_, scales2_,oris1_, oris2_, 3);

        HSolo hsolo;
        HSoloResult result = hsolo.findAffine(corrs, config);

        return prepareResult(result, get_stats);
    }
    catch (const char *msg)
    {
        throw std::invalid_argument(msg);
    }
}

py::tuple findHomography_(py::array_t<double> x1y1_,
                          py::array_t<double> x2y2_,
                          py::array_t<double> scales1_,
                          py::array_t<double> scales2_,
                          py::array_t<double> oris1_,
                          py::array_t<double> oris2_,
                          double threshold,
                          double filtered_inlier_rate,
                          int search_top_n_filtered,
                          double conf,
                          int max_iters,
                          double run_inner_ransac_thresh,
                          bool get_stats,
                          bool refine_solution)
{

    try
    {
        HSoloConfiguration config;
        config.setErrorThreshold(threshold);
        config.setConfidence(conf);
        config.setFilteredInlierRate(filtered_inlier_rate);
        config.setRunInnerRANSACThresh(run_inner_ransac_thresh);
        config.setMaxIterations(max_iters);
        config.setSearchTopNfiltered(search_top_n_filtered);
        config.setRefineSolution(refine_solution);

        std::vector<HSoloCorrespondence> corrs = prepareData(x1y1_, x2y2_, 
                                        scales1_, scales2_,oris1_, oris2_, 4);

        HSolo hsolo;
        HSoloResult result = hsolo.findHomography(corrs, config);

        return prepareResult(result, get_stats);
    }
    catch (const char *msg)
    {
        throw std::invalid_argument(msg);
    }
}


PYBIND11_MODULE(_pyhsolo, m)
{


    m.def("findHomography", &findHomography_, "Find a Homography from a set of n>=4 candidate correspondences.",
          py::arg("x1y1"),
          py::arg("x2y2"),
          py::arg("scales1"),
          py::arg("scales2"),
          py::arg("oris1"),
          py::arg("oris2"),
          py::arg("threshold") = 2.0,
          py::arg("filtered_inlier_rate") = 0.7,
          py::arg("search_top_n_filtered") = 20,
          py::arg("conf") = 0.95,
          py::arg("max_iters") = 1000,
          py::arg("run_inner_ransac_thresh") = 25.0,
          py::arg("get_stats") = true,
          py::arg("refine_solution") = true);

    m.def("findSimilarity", &findSimilairty_, "Find a Similarity Homography from a set of n>=2 candidate correspondences.",
          py::arg("x1y1"),
          py::arg("x2y2"),
          py::arg("scales1"),
          py::arg("scales2"),
          py::arg("oris1"),
          py::arg("oris2"),
          py::arg("threshold") = 2.0,
          py::arg("filtered_inlier_rate") = 0.7,
          py::arg("search_top_n_filtered") = 20,
          py::arg("conf") = 0.95,
          py::arg("max_iters") = 1000,
          py::arg("run_inner_ransac_thresh") = 25.0,
          py::arg("get_stats") = true,
          py::arg("refine_solution") = true);

    m.def("findAffine", &findAffine_, "Find a Affine Homography from a set of n>=3 candidate correspondences.",
          py::arg("x1y1"),
          py::arg("x2y2"),
          py::arg("scales1"),
          py::arg("scales2"),
          py::arg("oris1"),
          py::arg("oris2"),
          py::arg("threshold") = 2.0,
          py::arg("filtered_inlier_rate") = 0.7,
          py::arg("search_top_n_filtered") = 20,
          py::arg("conf") = 0.95,
          py::arg("max_iters") = 1000,
          py::arg("run_inner_ransac_thresh") = 25.0,
          py::arg("get_stats") = true,
          py::arg("refine_solution") = true);

    m.def("findTranslation", &findTranslation_, "Find a Translation from a set of n>=1 candidate correspondences.",
          py::arg("x1y1"),
          py::arg("x2y2"),
          py::arg("scales1"),
          py::arg("scales2"),
          py::arg("oris1"),
          py::arg("oris2"),
          py::arg("threshold") = 2.0,
          py::arg("filtered_inlier_rate") = 0.7,
          py::arg("search_top_n_filtered") = 20,
          py::arg("conf") = 0.95,
          py::arg("max_iters") = 1000,
          py::arg("run_inner_ransac_thresh") = 13.0,
          py::arg("low_det_limit") = 0.1,
          py::arg("high_det_limit") = 2000.0,
          py::arg("get_stats") = true,
          py::arg("refine_solution") = true);

}

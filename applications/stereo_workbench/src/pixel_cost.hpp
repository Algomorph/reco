/*
 * pixel_cost.hpp
 *
 *  Created on: Dec 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#pragma once

#include <opencv2/core.hpp>
#include <functional>

#include <reco/stereo_workbench/semiglobal_matcher.hpp>

namespace reco{
namespace stereo_workbench{



typedef uchar PixType;
typedef short CostType;
typedef short DispType;

typedef std::function<void (const cv::Mat&, const cv::Mat&, int, int, int,
		CostType*, PixType*, const PixType*, int, int)> row_cost_function;

typedef std::function<void (const cv::Mat&, const cv::Mat&, cv::Mat&, cv::Mat&)> cost_precomputation_function;

void calculate_row_cost_DAISY( const cv::Mat& img1, const cv::Mat& img2, int y,
                            int minD, int maxD, CostType* cost,
                            PixType* buffer, const PixType* tab,
                            int tabOfs, int);

void calculate_row_cost_BT( const cv::Mat& img1, const cv::Mat& img2, int y,
                            int minD, int maxD, CostType* cost,
                            PixType* buffer, const PixType* tab,
                            int tabOfs, int );
void calculate_row_cost_L2( const cv::Mat& img1, const cv::Mat& img2, int y,
        int minD, int maxD, CostType* cost,
        PixType* buffer, const PixType* lookup_table,
        int tabOfs, int);

void precompute_nothing(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& out_img1, cv::Mat& out_img2);
void precompute_DAISY(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& out_img1, cv::Mat& out_img2);
}//stereo_workbench
}//reco

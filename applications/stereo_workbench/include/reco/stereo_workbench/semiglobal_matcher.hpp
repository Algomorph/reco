/*
 * semiglobal_matcher.hpp
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

#include <opencv2/calib3d.hpp>

namespace reco{
namespace stereo_workbench{
enum pixel_cost_type{
	BIRCHFIELD_TOMASI = 0,
	DAISY = 1,
	NORM_L2 = 3
};

cv::Ptr<cv::StereoSGBM> create_semiglobal_matcher(int minDisparity, int numDisparities, int block_size,
                                 int P1, int P2, int disp12MaxDiff,
                                 int preFilterCap, int uniquenessRatio,
                                 int speckleWindowSize, int speckleRange,
                                 int mode,
								 pixel_cost_type cost_type = pixel_cost_type::BIRCHFIELD_TOMASI);

}//stereo_workbench
}//reco


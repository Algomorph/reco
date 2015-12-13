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

struct semiglobal_matcher_parameters{
    semiglobal_matcher_parameters(){
        minDisparity = numDisparities = 0;
        block_size = 0;
        P1 = P2 = 0;
        disp12MaxDiff = 0;
        preFilterCap = 0;
        uniquenessRatio = 0;
        speckleWindowSize = 0;
        speckleRange = 0;
        mode = cv::StereoSGBM::MODE_SGBM;
    }

    semiglobal_matcher_parameters( int _minDisparity, int _numDisparities, int block_window_size,
                      int _P1, int _P2, int _disp12MaxDiff, int _preFilterCap,
                      int _uniquenessRatio, int _speckleWindowSize, int _speckleRange,
                      int _mode ){
        minDisparity = _minDisparity;
        numDisparities = _numDisparities;
        this->block_size = block_window_size;
        P1 = _P1;
        P2 = _P2;
        disp12MaxDiff = _disp12MaxDiff;
        preFilterCap = _preFilterCap;
        uniquenessRatio = _uniquenessRatio;
        speckleWindowSize = _speckleWindowSize;
        speckleRange = _speckleRange;
        mode = _mode;
    }

    int minDisparity;
    int numDisparities;
    int block_size;
    int preFilterCap;
    int uniquenessRatio;
    int P1;
    int P2;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int mode;

};
}//stereo_workbench
}//reco


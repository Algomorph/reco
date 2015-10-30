/*
 * macher_qt_wrapper.cpp
 *
 *  Created on: Oct 29, 2015
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

#include <reco/stereo_workbench/matcher_qt_wrapper.hpp>

namespace reco {
namespace stereo_workbench {

template<class MATCHER>
matcher_qt_wrapper<MATCHER>::matcher_qt_wrapper(cv::Ptr<MATCHER> matcher):
	stereo_matcher(matcher){


}

template<class MATCHER>
matcher_qt_wrapper<MATCHER>::~matcher_qt_wrapper(){

}

template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::compute(const cv::Mat& left,const cv::Mat& right,cv::Mat& disparity){
	stereo_matcher->compute(left,right,disparity);
}

//===========================PARAMETER GETTERS======================================================

template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_bock_size() const{
	return stereo_matcher->getBlockSize();
}
template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_disparity_max_diff() const{
	return stereo_matcher->getDisp12MaxDiff();
}
template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_minimum_disparity() const{
	return stereo_matcher->getMinDisparity();
}
template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_num_disparities() const{
	return stereo_matcher->getNumDisparities();
}
template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_speckle_range() const{
	return stereo_matcher->getSpeckleRange();
}
template<class MATCHER>
int matcher_qt_wrapper<MATCHER>::get_speckle_window_size() const{
	return stereo_matcher->getSpeckleWindowSize();
}


//==================================================================================================
//===========================PARAMETER SETTER SLOTS=================================================
template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_minimum_disparity(int value) {
	stereo_matcher->setMinDisparity(value);
	emit parameters_changed();
}

template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_num_disparities(int value) {
	stereo_matcher->setNumDisparities(value - (value % 16));
	emit parameters_changed();
}

template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_block_size(int value) {
	int new_window_size = value + ((value + 1) % 2);
	stereo_matcher->setBlockSize(new_window_size);
	emit parameters_changed();
}

template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_speckle_window_size(int value) {
	stereo_matcher->setSpeckleWindowSize(value);
	emit parameters_changed();
}
template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_speckle_range(int value) {
	stereo_matcher->setSpeckleRange(value);
	emit parameters_changed();
}

template<class MATCHER>
void matcher_qt_wrapper<MATCHER>::set_disparity_max_diff(int value){
	stereo_matcher->setDisp12MaxDiff(value);
	emit parameters_changed();
}
//=======================END PARAMETER SETTER SLOTS=================================================
//==================================================================================================

} /* namespace stereo_workbench */
} /* namespace reco */

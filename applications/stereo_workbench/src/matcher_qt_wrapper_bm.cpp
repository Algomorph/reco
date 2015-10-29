/*
 * stereo_processor_bm.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/matcher_qt_wrapper_bm.hpp>

namespace reco {
namespace stereo_workbench {


matcher_qt_wrapper_bm::matcher_qt_wrapper_bm():
		matcher_qt_wrapper(cv::StereoBM::create(256, 3)){

}
int matcher_qt_wrapper_bm::get_pre_filter_cap() const{
	return stereo_matcher->getPreFilterCap();
}

int matcher_qt_wrapper_bm::get_pre_filter_size() const{
	return stereo_matcher->getPreFilterSize();
}
int matcher_qt_wrapper_bm::get_pre_filter_type() const{
	return stereo_matcher->getPreFilterType();
}
int matcher_qt_wrapper_bm::get_smaller_block_size() const{
	return stereo_matcher->getSmallerBlockSize();
}
int matcher_qt_wrapper_bm::get_texture_threshold() const{
	return stereo_matcher->getTextureThreshold();
}

int matcher_qt_wrapper_bm::get_uniqueness_ratio() const{
	return stereo_matcher->getUniquenessRatio();
}

void matcher_qt_wrapper_bm::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	emit parameters_changed();
}

void matcher_qt_wrapper_bm::set_pre_filter_size(int value){
	stereo_matcher->setPreFilterSize(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_pre_filter_type(int value){
	stereo_matcher->setPreFilterType(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_smaller_block_size(int value){
	stereo_matcher->setSmallerBlockSize(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_texture_threshold(int value){
	stereo_matcher->setTextureThreshold(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	emit parameters_changed();
}

matcher_qt_wrapper_bm::~matcher_qt_wrapper_bm(){}

} /* namespace stereo_workbench */
} /* namespace reco */

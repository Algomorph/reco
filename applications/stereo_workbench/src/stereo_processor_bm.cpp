/*
 * stereo_processor_bm.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/stereo_processor_bm.hpp>

namespace reco {
namespace stereo_workbench {

stereo_processor_bm::stereo_processor_bm(datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<rectifier>  rectifier_instance):
						stereo_processor(input_frame_buffer, output_frame_buffer,
								cv::StereoBM::create(256, 3), rectifier_instance){

}
int stereo_processor_bm::get_pre_filter_cap() const{
	return stereo_matcher->getPreFilterCap();
}

int stereo_processor_bm::get_pre_filter_size() const{
	return stereo_matcher->getPreFilterSize();
}
int stereo_processor_bm::get_pre_filter_type() const{
	return stereo_matcher->getPreFilterType();
}
int stereo_processor_bm::get_smaller_block_size() const{
	return stereo_matcher->getSmallerBlockSize();
}
int stereo_processor_bm::get_texture_threshold() const{
	return stereo_matcher->getTextureThreshold();
}

int stereo_processor_bm::get_uniqueness_ratio() const{
	return stereo_matcher->getUniquenessRatio();
}

void stereo_processor_bm::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	recompute_disparity_if_paused();
}

void stereo_processor_bm::set_pre_filter_size(int value){
	stereo_matcher->setPreFilterSize(value);
	recompute_disparity_if_paused();
}
void stereo_processor_bm::set_pre_filter_type(int value){
	stereo_matcher->setPreFilterType(value);
	recompute_disparity_if_paused();
}
void stereo_processor_bm::set_smaller_block_size(int value){
	stereo_matcher->setSmallerBlockSize(value);
	recompute_disparity_if_paused();
}
void stereo_processor_bm::set_texture_threshold(int value){
	stereo_matcher->setTextureThreshold(value);
	recompute_disparity_if_paused();
}
void stereo_processor_bm::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	recompute_disparity_if_paused();
}

stereo_processor_bm::~stereo_processor_bm(){}

} /* namespace stereo_workbench */
} /* namespace reco */

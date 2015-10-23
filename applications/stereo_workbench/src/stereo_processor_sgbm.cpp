/*
 * stereo_processor_sgbm.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/stereo_processor_sgbm.hpp>

namespace reco {
namespace stereo_workbench {

stereo_processor_sgbm::stereo_processor_sgbm(datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<rectifier> rectifier) :
		stereo_processor(input_frame_buffer, output_frame_buffer, rectifier) {

}

stereo_processor_sgbm::~stereo_processor_sgbm() {

}

void stereo_processor_sgbm::set_p1(int value) {
	if (value < stereo_matcher->getP2()) {
		stereo_matcher->setP1(value);
		recompute_disparity_if_paused();
	}
}

void stereo_processor_sgbm::set_p2(int value) {
	if (value > stereo_matcher->getP1()) {
		stereo_matcher->setP2(value);
		recompute_disparity_if_paused();
	}
}

void stereo_processor_sgbm::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	recompute_disparity_if_paused();
}

void stereo_processor_sgbm::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	recompute_disparity_if_paused();
}

} /* namespace stereo_workbench */
} /* namespace reco */
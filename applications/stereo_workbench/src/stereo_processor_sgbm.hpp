/*
 * stereo_processor_sgbm.h
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <src/stereo_processor.hpp>

namespace reco {
namespace stereo_workbench {

class stereo_processor_sgbm:
		public stereo_processor<cv::StereoSGBM> {
	Q_OBJECT
public:
	stereo_processor_sgbm(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<rectifier>  rectifier_instance = std::shared_ptr<rectifier>());
	virtual ~stereo_processor_sgbm();

public slots:
	void set_p1(int value);
	void set_p2(int value);
	void set_pre_filter_cap(int value);
	void set_uniqueness_ratio(int value);




};

} /* namespace stereo_workbench */
} /* namespace reco */

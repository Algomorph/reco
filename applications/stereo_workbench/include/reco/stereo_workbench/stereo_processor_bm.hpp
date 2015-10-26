/*
 * stereo_processor_bm.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/stereo_processor.hpp>

namespace reco {
namespace stereo_workbench {

class stereo_processor_bm:
		public stereo_processor<cv::StereoBM> {
	Q_OBJECT
public:
	stereo_processor_bm(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<rectifier>  rectifier_instance = std::shared_ptr<rectifier>());
	virtual ~stereo_processor_bm();
	int get_pre_filter_cap() const;
	int get_pre_filter_size() const;
	int get_pre_filter_type() const;
	int get_smaller_block_size() const;
	int get_texture_threshold() const;
	int get_uniqueness_ratio() const;

public slots:
	void set_pre_filter_cap(int value);
	void set_pre_filter_size(int value);
	void set_pre_filter_type(int value);
	void set_smaller_block_size(int value);
	void set_texture_threshold(int value);
	void set_uniqueness_ratio(int value);

};

} /* namespace stereo_workbench */
} /* namespace reco */

/*
 * stereo_processor_bm.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/matcher_qt_wrapper.hpp>

namespace reco {
namespace stereo_workbench {

class matcher_qt_wrapper_bm:
		public matcher_qt_wrapper<cv::StereoBM> {
	Q_OBJECT
public:
	matcher_qt_wrapper_bm();
	virtual ~matcher_qt_wrapper_bm();
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

/*
 * sgbm_tuning_panel.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <src/stereo_matcher_tuning_panel.hpp>

namespace reco {
namespace stereo_workbench {

class sgbm_tuning_panel:
		public stereo_matcher_tuning_panel<cv::StereoSGBM> {
public:
	sgbm_tuning_panel();
	virtual ~sgbm_tuning_panel();
private:
	QHBoxLayout* p1_horizontal_layout;
	QLabel* p1_label;
	QSpinBox* p1_spin_box;
	QSlider* p1_slider;

	QHBoxLayout* p2_horizontal_layout;
	QLabel* p2_label;
	QSpinBox* p2_spin_box;
	QSlider* p2_slider;

	QHBoxLayout* pre_filter_cap_horizontal_layout;
	QLabel* pre_filter_cap_label;
	QSpinBox* pre_filter_cap_spin_box;
	QSlider* pre_filter_cap_slider;

	QHBoxLayout* uniqueness_ration_horizontal_layout;
	QLabel* uniqueness_rato_label;
	QSpinBox* uniqueness_ratio_spin_box;
	QSlider* uniqueness_ratio_slider;
};

} /* namespace stereo_workbench */
} /* namespace reco */

/*
 * sgbm_tuning_panel.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/stereo_matcher_tuning_panel.hpp>
#include <reco/stereo_workbench/stereo_processor_sgbm.hpp>
namespace reco {
namespace stereo_workbench {

class sgbm_tuning_panel:
		public stereo_matcher_tuning_panel<stereo_processor_sgbm> {
public:
	sgbm_tuning_panel(QWidget* parent = NULL);
	virtual ~sgbm_tuning_panel();

	virtual void connect_specialized_controls(const stereo_processor_sgbm& processor);

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

	void construct_specialized_controls();
};

} /* namespace stereo_workbench */
} /* namespace reco */

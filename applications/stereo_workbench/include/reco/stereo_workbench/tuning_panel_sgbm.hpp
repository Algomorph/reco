/*
 * sgbm_tuning_panel.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/matcher_qt_wrapper_sgbm.hpp>
#include <reco/stereo_workbench/tuning_panel.hpp>
#include <QWidget>

namespace reco {
namespace stereo_workbench {

class tuning_panel_sgbm:
		public tuning_panel{
public:
	tuning_panel_sgbm(const matcher_qt_wrapper_sgbm& processor, QWidget* parent = NULL);
	virtual ~tuning_panel_sgbm();

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

	QHBoxLayout* uniqueness_ratio_horizontal_layout;
	QLabel* uniqueness_ratio_label;
	QSpinBox* uniqueness_ratio_spin_box;
	QSlider* uniqueness_ratio_slider;

	void construct_specialized_controls();
	void connect_specialized_controls(const matcher_qt_wrapper_sgbm& processor);
};

} /* namespace stereo_workbench */
} /* namespace reco */

/*
 * stereo_matcher_tuning_panel.h
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

//qt
#include <QWidget>
#include <QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QPushButton>

//std
#include <unordered_map>
#include <array>
#include <string>
#include <memory>

//local
#include <reco/stereo_workbench/stereo_processor.hpp>
#include <reco/stereo_workbench/matcher_qt_wrapper.hpp>
#include <reco/stereo_workbench/tuning_panel.hpp>

namespace reco {
namespace stereo_workbench {


class stereo_matcher_tuning_panel:
		public tuning_panel {

public:
	stereo_matcher_tuning_panel(QWidget* parent = NULL);
	virtual ~stereo_matcher_tuning_panel();

	void connect_stereo_processor(const stereo_processor& processor);

private:
	void swap_specialized_panel(tuning_panel* new_panel);

	tuning_panel* specialized_parameter_panel;
	QVBoxLayout* other_controls_vlayout;

	QHBoxLayout* block_size_horizontal_layout;
	QLabel* block_size_label;
	QSpinBox* block_size_spin_box;
	QSlider* block_size_slider;

	QHBoxLayout* disparity_max_diff_horizontal_layout;
	QLabel* disparity_max_diff_label;
	QSpinBox* disparity_max_diff_spin_box;
	QSlider* disparity_max_diff_slider;

	QHBoxLayout* minimum_disparity_horizontal_layout;
	QLabel* minimum_disparity_label;
	QSpinBox* minimum_disparity_spin_box;
	QSlider* minimum_disparity_slider;

	QHBoxLayout* number_of_disparities_horizontal_layout;
	QLabel* number_of_disparities_label;
	QSpinBox* number_of_disparities_spin_box;
	QSlider* number_of_disparities_slider;

	QHBoxLayout* speckle_window_size_horizontal_layout;
	QLabel* speckle_window_size_label;
	QSpinBox* speckle_window_size_spin_box;
	QSlider* speckle_window_size_slider;

	QHBoxLayout* speckle_range_horizontal_layout;
	QLabel* speckle_range_label;
	QSpinBox* speckle_range_spin_box;
	QSlider* speckle_range_slider;

	QHBoxLayout* v_offset_horizontal_layout;
	QLabel* v_offset_label;
	QSpinBox* v_offset_spin_box;
	QSlider* v_offset_slider;

	QHBoxLayout* rectification_horizontal_layout;
	QCheckBox* rectify_checkbox;
	QPushButton* save_current_button;

	void set_up_tuning_controls();


};


} /* namespace stereo_workbench */
} /* namespace reco */

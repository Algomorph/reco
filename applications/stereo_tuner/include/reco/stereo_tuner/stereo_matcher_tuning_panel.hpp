/*
 * stereo_matcher_tuning_panel.h
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

//qt
#include <reco/stereo_tuner/matcher_qt_wrapper.hpp>
#include <reco/stereo_tuner/stereo_processor.hpp>
#include <reco/stereo_tuner/tuning_panel.hpp>
#include <QWidget>
#include <QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>

//std
#include <unordered_map>
#include <array>
#include <string>
#include <memory>

//local

namespace reco {
namespace stereo_tuner {


class stereo_matcher_tuning_panel:
		public tuning_panel {
Q_OBJECT
public:
	stereo_matcher_tuning_panel(QWidget* parent = NULL);
	virtual ~stereo_matcher_tuning_panel();

	void connect_stereo_processor(stereo_processor& processor);
	enum matcher_type{
		sgbm = 0,
		bm = 1,
		bp = 2
	};

	std::array<std::shared_ptr<matcher_qt_wrapper_base>,3> matchers;

private:

	void swap_specialized_panel(tuning_panel* new_panel);

	stereo_processor* processor;
	matcher_qt_wrapper_base* matcher;
	tuning_panel* specialized_parameter_panel;

	QVBoxLayout* other_controls_vlayout;


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

	QComboBox* matcher_type_combo_box;


	void set_up_tuning_controls();
private slots:
	void on_matcher_type_combo_box_currentIndexChanged(int index);
	void connect_matcher(matcher_qt_wrapper_base* matcher);

};


} /* namespace stereo_tuner */
} /* namespace reco */

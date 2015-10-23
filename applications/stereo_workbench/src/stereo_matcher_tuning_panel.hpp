/*
 * stereo_matcher_tuning_panel.h
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

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

namespace reco {
namespace stereo_workbench {

template<typename PROC>
class stereo_matcher_tuning_panel:
		public QWidget {
	static_assert(
		std::is_base_of<reco::stereo_workbench::stereo_processor, PROC>::value,
		"PROC must be a descendant of reco::stereo_workbench::stereo_processor"
	);
public:
	stereo_matcher_tuning_panel();
	virtual ~stereo_matcher_tuning_panel();
protected:
	std::shared_ptr<PROC> processor;
	QVBoxLayout* tuning_controls_vlayout;
private:
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

/*
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
*/



	QHBoxLayout* rectification_horizontal_layout;
	QCheckBox* rectify_checkbox;
	QPushButton* save_current_button;

	void generate_tuning_controls();

};


} /* namespace stereo_workbench */
} /* namespace reco */

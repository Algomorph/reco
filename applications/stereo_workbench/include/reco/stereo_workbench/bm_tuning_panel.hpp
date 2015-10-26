/*
 * bm_tuning_panel.hpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/stereo_matcher_tuning_panel.hpp>
#include <reco/stereo_workbench/stereo_processor_bm.hpp>

namespace reco {
namespace stereo_workbench {

class bm_tuning_panel:
		public stereo_matcher_tuning_panel<stereo_processor_bm> {
public:
	bm_tuning_panel(QWidget* parent = NULL);
	virtual ~bm_tuning_panel();

private:
	QHBoxLayout* pre_filter_cap_horizontal_layout;
	QLabel* pre_filter_cap_label;
	QSpinBox* pre_filter_cap_spin_box;
	QSlider* pre_filter_cap_slider;

	QHBoxLayout* pre_filter_size_horizontal_layout;
	QLabel* pre_filter_size_label;
	QSpinBox* pre_filter_size_spin_box;
	QSlider* pre_filter_size_slider;

	QComboBox *pre_filter_type_combo_box;

//	QHBoxLayout* pre_filter_type_horizontal_layout;
//	QLabel* pre_filter_type_label;
//	QSpinBox* pre_filter_type_spin_box;
//	QSlider* pre_filter_type_slider;

	QHBoxLayout* smaller_block_size_horizontal_layout;
	QLabel* smaller_block_size_label;
	QSpinBox* smaller_block_size_spin_box;
	QSlider* smaller_block_size_slider;

	QHBoxLayout* texture_threshold_horizontal_layout;
	QLabel* texture_threshold_label;
	QSpinBox* texture_threshold_spin_box;
	QSlider* texture_threshold_slider;

	QHBoxLayout* uniqueness_ratio_horizontal_layout;
	QLabel* uniqueness_ratio_label;
	QSpinBox* uniqueness_ratio_spin_box;
	QSlider* uniqueness_ratio_slider;

	void construct_specialized_controls();
};

} /* namespace stereo_workbench */
} /* namespace reco */

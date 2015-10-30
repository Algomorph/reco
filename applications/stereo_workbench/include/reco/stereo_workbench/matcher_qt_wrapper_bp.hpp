/*
 * stereo_processor_bm.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/matcher_qt_wrapper.hpp>
#include <reco/stereo_workbench/tuning_panel.hpp>
#include <QComboBox>

namespace reco {
namespace stereo_workbench {

class matcher_qt_wrapper_bp:
		public matcher_qt_wrapper<cv::cuda::StereoBeliefPropagation> {
	Q_OBJECT
private:

	class tuning_panel_bp:
			public tuning_panel{
	public:
		tuning_panel_bp(
				const matcher_qt_wrapper_bp& matcher,
				QWidget* parent = NULL);
		virtual ~tuning_panel_bp();

	private:
		QHBoxLayout* block_size_horizontal_layout;
		QLabel* block_size_label;
		QSpinBox* block_size_spin_box;
		QSlider* block_size_slider;

		QHBoxLayout* pre_filter_cap_horizontal_layout;
		QLabel* pre_filter_cap_label;
		QSpinBox* pre_filter_cap_spin_box;
		QSlider* pre_filter_cap_slider;

		QHBoxLayout* pre_filter_size_horizontal_layout;
		QLabel* pre_filter_size_label;
		QSpinBox* pre_filter_size_spin_box;
		QSlider* pre_filter_size_slider;

		QComboBox* pre_filter_type_combo_box;

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
		void connect_specialized_controls(const matcher_qt_wrapper_bp& matcher);
	};
public:
	matcher_qt_wrapper_bp();
	virtual ~matcher_qt_wrapper_bp();

	virtual void compute(const cv::Mat& left,const cv::Mat& right, cv::Mat& disparity);

	//getters
	double get_data_weight() const;
	double get_disc_single_jump() const;
	double get_max_data_term() const;
	double get_max_disc_term() const;
	int get_msg_type() const;
	int get_num_iters() const;
	int get_num_levels() const;

public slots:
	void set_data_weight(double value);
	void set_disc_single_jump(double value);
	void set_max_data_term(double value);
	void set_max_disc_term(double value);
	void set_msg_type(int value);
	void set_num_iters(int value);
	void set_num_levels(int value);

};

} /* namespace stereo_workbench */
} /* namespace reco */

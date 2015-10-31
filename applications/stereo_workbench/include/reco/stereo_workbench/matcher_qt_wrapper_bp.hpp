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
#include <opencv2/cudastereo.hpp>
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

		QHBoxLayout* data_weight_horizontal_layout;
		QLabel* data_weight_label;
		QDoubleSpinBox* data_weight_spin_box;

		QHBoxLayout* disc_single_jump_horizontal_layout;
		QLabel* disc_single_jump_label;
		QDoubleSpinBox* disc_single_jump_spin_box;

		QHBoxLayout* max_data_term_horizontal_layout;
		QLabel* max_data_term_label;
		QDoubleSpinBox* max_data_term_spin_box;

		QHBoxLayout* max_disc_term_horizontal_layout;
		QLabel* max_disc_term_label;
		QDoubleSpinBox* max_disc_term_spin_box;

		QComboBox* msg_type_combo_box;

		QHBoxLayout* num_iters_horizontal_layout;
		QLabel* num_iters_label;
		QSpinBox* num_iters_spin_box;
		QSlider* num_iters_slider;

		QHBoxLayout* num_levels_horizontal_layout;
		QLabel* num_levels_label;
		QSpinBox* num_levels_spin_box;
		QSlider* num_levels_slider;

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
	void set_msg_type(int combo_box_index);
	void set_num_iters(int value);
	void set_num_levels(int value);

};

} /* namespace stereo_workbench */
} /* namespace reco */

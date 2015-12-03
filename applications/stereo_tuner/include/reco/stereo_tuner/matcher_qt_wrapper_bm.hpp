/*
 * stereo_processor_bm.h
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_tuner/matcher_qt_wrapper.hpp>
#include <reco/stereo_tuner/tuning_panel.hpp>
#include <QComboBox>

namespace reco {
namespace stereo_tuner {

class matcher_qt_wrapper_bm:
		public matcher_qt_wrapper<cv::StereoBM> {
	Q_OBJECT
private:

	class tuning_panel_bm:
			public tuning_panel{
	public:
		tuning_panel_bm(
				const matcher_qt_wrapper_bm& matcher,
				QWidget* parent = NULL);
		virtual ~tuning_panel_bm();

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
		void connect_specialized_controls(const matcher_qt_wrapper_bm& matcher);
	};
public:
	matcher_qt_wrapper_bm();
	virtual ~matcher_qt_wrapper_bm();

	virtual void compute(const cv::Mat& left,const cv::Mat& right, cv::Mat& disparity);

	//getters
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

} /* namespace stereo_tuner */
} /* namespace reco */

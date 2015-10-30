/*
 * stereo_processor_sgbm.h
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo_workbench/matcher_qt_wrapper.hpp>
#include <reco/stereo_workbench/tuning_panel.hpp>
namespace reco {
namespace stereo_workbench {

class matcher_qt_wrapper_sgbm:
		public matcher_qt_wrapper<cv::StereoSGBM> {
	Q_OBJECT
private:
	class tuning_panel_sgbm:
			public tuning_panel{
	public:
		tuning_panel_sgbm(
				const matcher_qt_wrapper_sgbm& matcher,
				QWidget* parent = NULL);
		virtual ~tuning_panel_sgbm();

	private:
		QHBoxLayout* block_size_horizontal_layout;
		QLabel* block_size_label;
		QSpinBox* block_size_spin_box;
		QSlider* block_size_slider;

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
public:
	matcher_qt_wrapper_sgbm();
	virtual ~matcher_qt_wrapper_sgbm();

	//getters
	int get_p1() const;
	int get_p2() const;
	int get_pre_filter_cap() const;
	int get_uniqueness_ratio() const;

public slots:
	void set_p1(int value);
	void set_p2(int value);
	void set_pre_filter_cap(int value);
	void set_uniqueness_ratio(int value);
};

} /* namespace stereo_workbench */
} /* namespace reco */

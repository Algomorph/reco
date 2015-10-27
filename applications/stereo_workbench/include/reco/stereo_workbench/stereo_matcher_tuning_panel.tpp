/*
 * stereo_matcher_tuning_panel.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/stereo_matcher_tuning_panel.hpp>
#include <QVBoxLayout>

namespace reco {
namespace stereo_workbench {

template<typename PROC>
stereo_matcher_tuning_panel<PROC>::stereo_matcher_tuning_panel(QWidget* parent):
	QWidget(parent){

	this->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	this->setMinimumSize(100,100);
	QVBoxLayout* layout = new QVBoxLayout();
	this->setLayout(layout);
	tuning_controls_vlayout = new QVBoxLayout();
	other_controls_vlayout = new QVBoxLayout();
	layout->addItem(tuning_controls_vlayout);
	layout->addItem(other_controls_vlayout);

	set_up_tuning_controls();
}



template<typename PROC>
void stereo_matcher_tuning_panel<PROC>
	::connect_standard_controls(const PROC& processor){
	rectify_checkbox->setChecked(processor.is_rectification_enabled());
	//====================== SLIDER / SPINNER CONTROLS =================================================
	//sliders
	minimum_disparity_slider->      setValue(processor.get_minimum_disparity());
	number_of_disparities_slider->  setValue(processor.get_num_disparities());
	block_size_slider->             setValue(processor.get_bock_size());
	speckle_window_size_slider->    setValue(processor.get_speckle_window_size());
	speckle_range_slider->          setValue(processor.get_speckle_range());
	v_offset_slider->               setValue(processor.get_v_offset());

	//spin-boxes
	minimum_disparity_spin_box->    setValue(processor.get_minimum_disparity());
	number_of_disparities_spin_box->setValue(processor.get_num_disparities());
	block_size_spin_box->           setValue(processor.get_bock_size());
	speckle_window_size_spin_box->  setValue(processor.get_speckle_window_size());
	speckle_range_spin_box->        setValue(processor.get_speckle_range());
	v_offset_spin_box->             setValue(processor.get_v_offset());

	connect(minimum_disparity_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_minimum_disparity(int)));
	connect(minimum_disparity_slider, SIGNAL(valueChanged(int)), minimum_disparity_spin_box, SLOT(setValue(int)));
	connect(minimum_disparity_spin_box, SIGNAL(valueChanged(int)), minimum_disparity_slider, SLOT(setValue(int)));
	connect(number_of_disparities_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_num_disparities(int)));
	connect(number_of_disparities_slider, SIGNAL(valueChanged(int)), number_of_disparities_spin_box, SLOT(setValue(int)));
	connect(number_of_disparities_spin_box, SIGNAL(valueChanged(int)), number_of_disparities_slider, SLOT(setValue(int)));
	connect(block_size_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_block_size(int)));
	connect(block_size_slider, SIGNAL(valueChanged(int)), block_size_spin_box, SLOT(setValue(int)));
	connect(block_size_spin_box, SIGNAL(valueChanged(int)), block_size_slider, SLOT(setValue(int)));
	connect(speckle_window_size_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_speckle_window_size(int)));
	connect(speckle_window_size_slider, SIGNAL(valueChanged(int)), speckle_window_size_spin_box, SLOT(setValue(int)));
	connect(speckle_window_size_spin_box, SIGNAL(valueChanged(int)), speckle_window_size_slider, SLOT(setValue(int)));
	connect(speckle_range_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_speckle_range(int)));
	connect(speckle_range_slider, SIGNAL(valueChanged(int)), speckle_range_spin_box, SLOT(setValue(int)));
	connect(speckle_range_spin_box, SIGNAL(valueChanged(int)), speckle_range_slider, SLOT(setValue(int)));
	connect(v_offset_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_v_offset(int)));
	connect(v_offset_slider, SIGNAL(valueChanged(int)), v_offset_spin_box, SLOT(setValue(int)));
	connect(v_offset_spin_box, SIGNAL(valueChanged(int)), v_offset_slider, SLOT(setValue(int)));

}

template<typename PROC>
void stereo_matcher_tuning_panel<PROC>::construct_integer_control_set(
			QHBoxLayout* horizontal_layout,
			QLabel* label,
			QSpinBox* spin_box,
			QSlider* slider,
			QString layout_name,
			QString label_name,
			QString spin_box_name,
			QString slider_name,
			int min_val, int max_val, int step, int val, int page_step){
	horizontal_layout = new QHBoxLayout();
	horizontal_layout->setObjectName(QStringLiteral("horizontal_layout"));
	label = new QLabel(this);
	label->setObjectName(QStringLiteral("label"));
	label->setText("block size (pixels):");

	horizontal_layout->addWidget(label);

	spin_box = new QSpinBox(this);
	spin_box->setObjectName(QStringLiteral("spin_box"));
	spin_box->setMinimum(min_val);
	spin_box->setMaximum(max_val);
	spin_box->setSingleStep(step);


	horizontal_layout->addWidget(spin_box);

	tuning_controls_vlayout->addLayout(horizontal_layout);

	slider = new QSlider(this);
	slider->setObjectName(QStringLiteral("slider"));
	QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
	sizePolicy.setHorizontalStretch(0);
	sizePolicy.setVerticalStretch(0);
	sizePolicy.setHeightForWidth(slider->sizePolicy().hasHeightForWidth());
	slider->setSizePolicy(sizePolicy);
	slider->setMinimum(min_val);
	slider->setMaximum(max_val);
	slider->setSingleStep(step);
	slider->setPageStep(page_step);
	slider->setValue(val);
	slider->setTracking(false);
	slider->setOrientation(Qt::Horizontal);
	slider->setToolTip("block size (pixels)");
	tuning_controls_vlayout->addWidget(slider);
}

template<typename PROC>
void stereo_matcher_tuning_panel<PROC>
	::connect_to_stereo_processor(const PROC& processor){
	connect_standard_controls(processor);
	connect_specialized_controls(processor);
}

template<typename PROC>
void stereo_matcher_tuning_panel<PROC>::set_up_tuning_controls(){

	construct_integer_control_set(
			block_size_horizontal_layout,
			block_size_label,
			block_size_spin_box,
			block_size_slider,
			QStringLiteral("block_size_horizontal_layout"),
			QStringLiteral("block_size_label"),
			QStringLiteral("block_size_spin_box"),
			QStringLiteral("block_size_slider"),
			1,65,2,1
			);

	construct_integer_control_set(
			disparity_max_diff_horizontal_layout,
			disparity_max_diff_label,
			disparity_max_diff_spin_box,
			disparity_max_diff_slider,
			QStringLiteral("disparity_max_diff_horizontal_layout"),
			QStringLiteral("disparity_max_diff_label"),
			QStringLiteral("disparity_max_diff_spin_box"),
			QStringLiteral("disparity_max_diff_slider"),
			0,256
			);

	construct_integer_control_set(
			minimum_disparity_horizontal_layout,
			minimum_disparity_label,
			minimum_disparity_spin_box,
			minimum_disparity_slider,
			QStringLiteral("minimum_disparity_horizontal_layout"),
			QStringLiteral("minimum_disparity_label"),
			QStringLiteral("minimum_disparity_spin_box"),
			QStringLiteral("minimum_disparity_slider"),
			0,256
			);

	construct_integer_control_set(
			number_of_disparities_horizontal_layout,
			number_of_disparities_label,
			number_of_disparities_spin_box,
			number_of_disparities_slider,
			QStringLiteral("number_of_disparities_horizontal_layout"),
			QStringLiteral("number_of_disparities_label"),
			QStringLiteral("number_of_disparities_spin_box"),
			QStringLiteral("number_of_disparities_slider"),
			16,256,16,16
			);

	construct_integer_control_set(
			speckle_range_horizontal_layout,
			speckle_range_label,
			speckle_range_spin_box,
			speckle_range_slider,
			QStringLiteral("speckle_range_horizontal_layout"),
			QStringLiteral("speckle_range_label"),
			QStringLiteral("speckle_range_spin_box"),
			QStringLiteral("speckle_range_slider"),
			0,5
			);

	construct_integer_control_set(
			speckle_window_size_horizontal_layout,
			speckle_window_size_label,
			speckle_window_size_spin_box,
			speckle_window_size_slider,
			QStringLiteral("speckle_window_size_horizontal_layout"),
			QStringLiteral("speckle_window_size_label"),
			QStringLiteral("speckle_window_size_spin_box"),
			QStringLiteral("speckle_window_size_slider"),
			0,300,5,0,20
			);

	construct_integer_control_set(
			v_offset_horizontal_layout,
			v_offset_label,
			v_offset_spin_box,
			v_offset_slider,
			QStringLiteral("v_offset_horizontal_layout"),
			QStringLiteral("v_offset_label"),
			QStringLiteral("v_offset_spin_box"),
			QStringLiteral("v_offset_slider"),
			-128,128
			);

	rectification_horizontal_layout = new QHBoxLayout();
	rectification_horizontal_layout->setObjectName(QStringLiteral("rectification_horizontal_layout"));
	rectify_checkbox = new QCheckBox(this);
	rectify_checkbox->setObjectName(QStringLiteral("rectify_checkbox"));
	rectify_checkbox->setChecked(true);
	rectify_checkbox->setText("enable rectification");

	rectification_horizontal_layout->addWidget(rectify_checkbox);


	other_controls_vlayout->addLayout(rectification_horizontal_layout);

	save_current_button = new QPushButton(this);
	save_current_button->setObjectName(QStringLiteral("save_current_button"));
	save_current_button->setText("Save current");
	save_current_button->setToolTip("save current stereo matcher input");

	other_controls_vlayout->addWidget(save_current_button);
}


template<typename PROC>
stereo_matcher_tuning_panel<PROC>::~stereo_matcher_tuning_panel(){

}

} /* namespace stereo_workbench */
} /* namespace reco */

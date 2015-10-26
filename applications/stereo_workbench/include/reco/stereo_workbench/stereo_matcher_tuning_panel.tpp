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
void stereo_matcher_tuning_panel<PROC>
	::connect_to_stereo_processor(const PROC& processor){
	connect_standard_controls(processor);
	connect_specialized_controls(processor);
}


template<typename PROC>
void stereo_matcher_tuning_panel<PROC>::set_up_tuning_controls(){
	block_size_horizontal_layout = new QHBoxLayout();
	block_size_horizontal_layout->setObjectName(QStringLiteral("block_size_horizontal_layout"));
	block_size_label = new QLabel(this);
	block_size_label->setObjectName(QStringLiteral("block_size_label"));

	block_size_horizontal_layout->addWidget(block_size_label);

	block_size_spin_box = new QSpinBox(this);
	block_size_spin_box->setObjectName(QStringLiteral("block_size_spin_box"));
	block_size_spin_box->setMinimum(1);
	block_size_spin_box->setMaximum(65);
	block_size_spin_box->setSingleStep(2);

	block_size_horizontal_layout->addWidget(block_size_spin_box);


	tuning_controls_vlayout->addLayout(block_size_horizontal_layout);

	block_size_slider = new QSlider(this);
	block_size_slider->setObjectName(QStringLiteral("block_size_slider"));
	QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
	sizePolicy1.setHorizontalStretch(0);
	sizePolicy1.setVerticalStretch(0);
	sizePolicy1.setHeightForWidth(block_size_slider->sizePolicy().hasHeightForWidth());
	block_size_slider->setSizePolicy(sizePolicy1);
	block_size_slider->setMinimum(1);
	block_size_slider->setMaximum(65);
	block_size_slider->setSingleStep(2);
	block_size_slider->setValue(1);
	block_size_slider->setTracking(false);
	block_size_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(block_size_slider);

	disparity_max_diff_horizontal_layout = new QHBoxLayout();
	disparity_max_diff_horizontal_layout->setObjectName(QStringLiteral("disparity_max_diff_horizontal_layout"));
	disparity_max_diff_label = new QLabel(this);
	disparity_max_diff_label->setObjectName(QStringLiteral("disparity_max_diff_label"));

	disparity_max_diff_horizontal_layout->addWidget(disparity_max_diff_label);

	disparity_max_diff_spin_box = new QSpinBox(this);
	disparity_max_diff_spin_box->setObjectName(QStringLiteral("disparity_max_diff_spin_box"));
	disparity_max_diff_spin_box->setMaximum(256);

	disparity_max_diff_horizontal_layout->addWidget(disparity_max_diff_spin_box);

	disparity_max_diff_slider = new QSlider(this);
	disparity_max_diff_slider->setObjectName(QStringLiteral("disparity_max_diff_slider"));
	sizePolicy1.setHeightForWidth(disparity_max_diff_slider->sizePolicy().hasHeightForWidth());
	disparity_max_diff_slider->setSizePolicy(sizePolicy1);
	disparity_max_diff_slider->setMinimumSize(QSize(400, 0));
	disparity_max_diff_slider->setMaximum(256);
	disparity_max_diff_slider->setTracking(false);
	disparity_max_diff_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(disparity_max_diff_slider);

	tuning_controls_vlayout->addLayout(disparity_max_diff_horizontal_layout);

	minimum_disparity_horizontal_layout = new QHBoxLayout();
	minimum_disparity_horizontal_layout->setObjectName(QStringLiteral("minimum_disparity_horizontal_layout"));
	minimum_disparity_label = new QLabel(this);
	minimum_disparity_label->setObjectName(QStringLiteral("minimum_disparity_label"));

	minimum_disparity_horizontal_layout->addWidget(minimum_disparity_label);

	minimum_disparity_spin_box = new QSpinBox(this);
	minimum_disparity_spin_box->setObjectName(QStringLiteral("minimum_disparity_spin_box"));
	minimum_disparity_spin_box->setMaximum(256);

	minimum_disparity_horizontal_layout->addWidget(minimum_disparity_spin_box);


	tuning_controls_vlayout->addLayout(minimum_disparity_horizontal_layout);

	minimum_disparity_slider = new QSlider(this);
	minimum_disparity_slider->setObjectName(QStringLiteral("minimum_disparity_slider"));
	sizePolicy1.setHeightForWidth(minimum_disparity_slider->sizePolicy().hasHeightForWidth());
	minimum_disparity_slider->setSizePolicy(sizePolicy1);
	minimum_disparity_slider->setMinimumSize(QSize(400, 0));
	minimum_disparity_slider->setMaximum(256);
	minimum_disparity_slider->setTracking(false);
	minimum_disparity_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(minimum_disparity_slider);

	number_of_disparities_horizontal_layout = new QHBoxLayout();
	number_of_disparities_horizontal_layout->setObjectName(QStringLiteral("number_of_disparities_horizontal_layout"));
	number_of_disparities_label = new QLabel(this);
	number_of_disparities_label->setObjectName(QStringLiteral("number_of_disparities_label"));

	number_of_disparities_horizontal_layout->addWidget(number_of_disparities_label);

	number_of_disparities_spin_box = new QSpinBox(this);
	number_of_disparities_spin_box->setObjectName(QStringLiteral("number_of_disparities_spin_box"));
	number_of_disparities_spin_box->setMinimum(8);
	number_of_disparities_spin_box->setMaximum(256);
	number_of_disparities_spin_box->setSingleStep(16);

	number_of_disparities_horizontal_layout->addWidget(number_of_disparities_spin_box);


	tuning_controls_vlayout->addLayout(number_of_disparities_horizontal_layout);

	number_of_disparities_slider = new QSlider(this);
	number_of_disparities_slider->setObjectName(QStringLiteral("number_of_disparities_slider"));
	sizePolicy1.setHeightForWidth(number_of_disparities_slider->sizePolicy().hasHeightForWidth());
	number_of_disparities_slider->setSizePolicy(sizePolicy1);
	number_of_disparities_slider->setMinimum(8);
	number_of_disparities_slider->setMaximum(256);
	number_of_disparities_slider->setSingleStep(16);
	number_of_disparities_slider->setPageStep(32);
	number_of_disparities_slider->setTracking(false);
	number_of_disparities_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(number_of_disparities_slider);

	speckle_range_horizontal_layout = new QHBoxLayout();
	speckle_range_horizontal_layout->setObjectName(QStringLiteral("speckle_range_horizontal_layout"));
	speckle_range_label = new QLabel(this);
	speckle_range_label->setObjectName(QStringLiteral("speckle_range_label"));

	speckle_range_horizontal_layout->addWidget(speckle_range_label);

	speckle_range_spin_box = new QSpinBox(this);
	speckle_range_spin_box->setObjectName(QStringLiteral("speckle_range_spin_box"));
	speckle_range_spin_box->setMaximum(5);

	speckle_range_horizontal_layout->addWidget(speckle_range_spin_box);


	tuning_controls_vlayout->addLayout(speckle_range_horizontal_layout);

	speckle_range_slider = new QSlider(this);
	speckle_range_slider->setObjectName(QStringLiteral("speckle_range_slider"));
	sizePolicy1.setHeightForWidth(speckle_range_slider->sizePolicy().hasHeightForWidth());
	speckle_range_slider->setSizePolicy(sizePolicy1);
	speckle_range_slider->setMaximum(5);
	speckle_range_slider->setPageStep(2);
	speckle_range_slider->setTracking(false);
	speckle_range_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(speckle_range_slider);

	speckle_window_size_horizontal_layout = new QHBoxLayout();
	speckle_window_size_horizontal_layout->setObjectName(QStringLiteral("speckle_window_size_horizontal_layout"));
	speckle_window_size_label = new QLabel(this);
	speckle_window_size_label->setObjectName(QStringLiteral("speckle_window_size_label"));

	speckle_window_size_horizontal_layout->addWidget(speckle_window_size_label);

	speckle_window_size_spin_box = new QSpinBox(this);
	speckle_window_size_spin_box->setObjectName(QStringLiteral("speckle_window_size_spin_box"));
	speckle_window_size_spin_box->setMaximum(300);

	speckle_window_size_horizontal_layout->addWidget(speckle_window_size_spin_box);


	tuning_controls_vlayout->addLayout(speckle_window_size_horizontal_layout);

	speckle_window_size_slider = new QSlider(this);
	speckle_window_size_slider->setObjectName(QStringLiteral("speckle_window_size_slider"));
	sizePolicy1.setHeightForWidth(speckle_window_size_slider->sizePolicy().hasHeightForWidth());
	speckle_window_size_slider->setSizePolicy(sizePolicy1);
	speckle_window_size_slider->setMaximum(300);
	speckle_window_size_slider->setSingleStep(5);
	speckle_window_size_slider->setPageStep(20);
	speckle_window_size_slider->setTracking(false);
	speckle_window_size_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(speckle_window_size_slider);

	v_offset_horizontal_layout = new QHBoxLayout();
	v_offset_horizontal_layout->setObjectName(QStringLiteral("v_offset_horizontal_layout"));
	v_offset_label = new QLabel(this);
	v_offset_label->setObjectName(QStringLiteral("v_offset_label"));

	v_offset_horizontal_layout->addWidget(v_offset_label);

	v_offset_spin_box = new QSpinBox(this);
	v_offset_spin_box->setObjectName(QStringLiteral("v_offset_spin_box"));
	v_offset_spin_box->setMinimum(-128);
	v_offset_spin_box->setMaximum(128);

	v_offset_horizontal_layout->addWidget(v_offset_spin_box);


	tuning_controls_vlayout->addLayout(v_offset_horizontal_layout);

	v_offset_slider = new QSlider(this);
	v_offset_slider->setObjectName(QStringLiteral("v_offset_slider"));
	sizePolicy1.setHeightForWidth(v_offset_slider->sizePolicy().hasHeightForWidth());
	v_offset_slider->setSizePolicy(sizePolicy1);
	v_offset_slider->setMinimum(-128);
	v_offset_slider->setMaximum(128);
	v_offset_slider->setTracking(false);
	v_offset_slider->setOrientation(Qt::Horizontal);

	rectification_horizontal_layout = new QHBoxLayout();
	rectification_horizontal_layout->setObjectName(QStringLiteral("rectification_horizontal_layout"));
	rectify_checkbox = new QCheckBox(this);
	rectify_checkbox->setObjectName(QStringLiteral("rectify_checkbox"));
	rectify_checkbox->setChecked(true);

	rectification_horizontal_layout->addWidget(rectify_checkbox);


	other_controls_vlayout->addLayout(rectification_horizontal_layout);

	save_current_button = new QPushButton(this);
	save_current_button->setObjectName(QStringLiteral("save_current_button"));

	other_controls_vlayout->addWidget(save_current_button);
}


template<typename PROC>
stereo_matcher_tuning_panel<PROC>::~stereo_matcher_tuning_panel(){

}

} /* namespace stereo_workbench */
} /* namespace reco */

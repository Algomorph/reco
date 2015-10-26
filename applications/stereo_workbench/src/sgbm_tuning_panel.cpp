/*
 * sgbm_tuning_panel.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/sgbm_tuning_panel.h>

namespace reco {
namespace stereo_workbench {

sgbm_tuning_panel::sgbm_tuning_panel():stereo_matcher_tuning_panel<cv::StereoSGBM>(){
	QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
	p1_horizontal_layout = new QHBoxLayout();
	p1_horizontal_layout->setObjectName(QStringLiteral("p1_horizontal_layout"));
	p1_label = new QLabel(this);
	p1_label->setObjectName(QStringLiteral("p1_label"));

	p1_horizontal_layout->addWidget(p1_label);

	p1_spin_box = new QSpinBox(this);
	p1_spin_box->setObjectName(QStringLiteral("p1_spin_box"));
	p1_spin_box->setMinimum(0);
	p1_spin_box->setMaximum(2904);
	p1_spin_box->setSingleStep(12);

	p1_horizontal_layout->addWidget(p1_spin_box);


	tuning_controls_vlayout->addLayout(p1_horizontal_layout);

	p1_slider = new QSlider(this);
	p1_slider->setObjectName(QStringLiteral("p1_slider"));
	sizePolicy1.setHeightForWidth(p1_slider->sizePolicy().hasHeightForWidth());
	p1_slider->setSizePolicy(sizePolicy1);
	p1_slider->setMinimum(0);
	p1_slider->setMaximum(2904);
	p1_slider->setSingleStep(24);
	p1_slider->setPageStep(960);
	p1_slider->setTracking(false);
	p1_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(p1_slider);

	p2_horizontal_layout = new QHBoxLayout();
	p2_horizontal_layout->setObjectName(QStringLiteral("p2_horizontal_layout"));
	p2_label = new QLabel(this);
	p2_label->setObjectName(QStringLiteral("p2_label"));

	p2_horizontal_layout->addWidget(p2_label);

	p2_spin_box = new QSpinBox(this);
	p2_spin_box->setObjectName(QStringLiteral("p2_spin_box"));
	p2_spin_box->setMinimum(0);
	p2_spin_box->setMaximum(11616);
	p2_spin_box->setSingleStep(12);

	p2_horizontal_layout->addWidget(p2_spin_box);


	tuning_controls_vlayout->addLayout(p2_horizontal_layout);

	p2_slider = new QSlider(this);
	p2_slider->setObjectName(QStringLiteral("p2_slider"));
	sizePolicy1.setHeightForWidth(p2_slider->sizePolicy().hasHeightForWidth());
	p2_slider->setSizePolicy(sizePolicy1);
	p2_slider->setMinimum(0);
	p2_slider->setMaximum(11616);
	p2_slider->setSingleStep(24);
	p2_slider->setPageStep(960);
	p2_slider->setTracking(false);
	p2_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(p2_slider);

	pre_filter_cap_horizontal_layout = new QHBoxLayout();
	pre_filter_cap_horizontal_layout->setObjectName(QStringLiteral("horizontalLayout_2"));
	pre_filter_cap_label = new QLabel(this);
	pre_filter_cap_label->setObjectName(QStringLiteral("pre_filter_cap_label"));

	pre_filter_cap_horizontal_layout->addWidget(pre_filter_cap_label);

	pre_filter_cap_spin_box = new QSpinBox(this);
	pre_filter_cap_spin_box->setObjectName(QStringLiteral("pre_filter_cap_spin_box"));
	pre_filter_cap_spin_box->setMaximum(256);

	pre_filter_cap_horizontal_layout->addWidget(pre_filter_cap_spin_box);


	tuning_controls_vlayout->addLayout(pre_filter_cap_horizontal_layout);

	pre_filter_cap_slider = new QSlider(this);
	pre_filter_cap_slider->setObjectName(QStringLiteral("pre_filter_cap_slider"));
	sizePolicy1.setHeightForWidth(pre_filter_cap_slider->sizePolicy().hasHeightForWidth());
	pre_filter_cap_slider->setSizePolicy(sizePolicy1);
	pre_filter_cap_slider->setMaximum(256);
	pre_filter_cap_slider->setTracking(false);
	pre_filter_cap_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(pre_filter_cap_slider);

	uniqueness_ration_horizontal_layout = new QHBoxLayout();
	uniqueness_ration_horizontal_layout->setObjectName(QStringLiteral("uniqueness_ration_horizontal_layout"));
	uniqueness_rato_label = new QLabel(this);
	uniqueness_rato_label->setObjectName(QStringLiteral("uniqueness_rato_label"));

	uniqueness_ration_horizontal_layout->addWidget(uniqueness_rato_label);

	uniqueness_ratio_spin_box = new QSpinBox(this);
	uniqueness_ratio_spin_box->setObjectName(QStringLiteral("uniqueness_ratio_spin_box"));
	uniqueness_ratio_spin_box->setMaximum(20);

	uniqueness_ration_horizontal_layout->addWidget(uniqueness_ratio_spin_box);


	tuning_controls_vlayout->addLayout(uniqueness_ration_horizontal_layout);

	uniqueness_ratio_slider = new QSlider(this);
	uniqueness_ratio_slider->setObjectName(QStringLiteral("uniqueness_ratio_slider"));
	sizePolicy1.setHeightForWidth(uniqueness_ratio_slider->sizePolicy().hasHeightForWidth());
	uniqueness_ratio_slider->setSizePolicy(sizePolicy1);
	uniqueness_ratio_slider->setMaximum(20);
	uniqueness_ratio_slider->setTracking(false);
	uniqueness_ratio_slider->setOrientation(Qt::Horizontal);

	tuning_controls_vlayout->addWidget(uniqueness_ratio_slider);
}

sgbm_tuning_panel::~sgbm_tuning_panel(){

}

} /* namespace stereo_workbench */
} /* namespace reco */

/*
 * sgbm_tuning_panel.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/sgbm_tuning_panel.hpp>

namespace reco {
namespace stereo_workbench {

sgbm_tuning_panel::sgbm_tuning_panel(QWidget* parent)
	:stereo_matcher_tuning_panel<stereo_processor_sgbm>(parent){
	construct_specialized_controls();
}


void sgbm_tuning_panel::connect_specialized_controls(const stereo_processor_sgbm& processor){
	p1_slider->                     setValue(processor.get_p1());
	p2_slider->                     setValue(processor.get_p2());
	pre_filter_cap_slider->         setValue(processor.get_pre_filter_cap());
	uniqueness_ratio_slider->       setValue(processor.get_uniqueness_ratio());

	p1_spin_box->                   setValue(processor.get_p1());
	p2_spin_box->                   setValue(processor.get_p2());
	pre_filter_cap_spin_box->       setValue(processor.get_pre_filter_cap());
	uniqueness_ratio_spin_box->     setValue(processor.get_uniqueness_ratio());

	connect(p1_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_p1(int)));
	connect(p1_slider, SIGNAL(valueChanged(int)), p1_spin_box, SLOT(setValue(int)));
	connect(p1_spin_box, SIGNAL(valueChanged(int)), p1_slider, SLOT(setValue(int)));
	connect(p2_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_p2(int)));
	connect(p2_slider, SIGNAL(valueChanged(int)), p2_spin_box, SLOT(setValue(int)));
	connect(p2_spin_box, SIGNAL(valueChanged(int)), p2_slider, SLOT(setValue(int)));
	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_pre_filter_cap(int)));
	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), pre_filter_cap_spin_box, SLOT(setValue(int)));
	connect(pre_filter_cap_spin_box, SIGNAL(valueChanged(int)), pre_filter_cap_slider, SLOT(setValue(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_uniqueness_ratio(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), uniqueness_ratio_spin_box, SLOT(setValue(int)));
	connect(uniqueness_ratio_spin_box, SIGNAL(valueChanged(int)), uniqueness_ratio_slider, SLOT(setValue(int)));
}

void sgbm_tuning_panel::construct_specialized_controls(){
	QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
	p1_horizontal_layout = new QHBoxLayout();
	p1_horizontal_layout->setObjectName(QStringLiteral("p1_horizontal_layout"));
	p1_label = new QLabel(this);
	p1_label->setObjectName(QStringLiteral("p1_label"));
	p1_label->setText("p1:");

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
	p2_label->setText("p2:");

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
	pre_filter_cap_label->setText("pre-filter cap:");

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
	uniqueness_ratio_label = new QLabel(this);
	uniqueness_ratio_label->setObjectName(QStringLiteral("uniqueness_ratio_label"));
	uniqueness_ratio_label->setText("uniqueness ratio");

	uniqueness_ration_horizontal_layout->addWidget(uniqueness_ratio_label);

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

/*
 * stereo_processor_sgbm.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/matcher_qt_wrapper_sgbm.hpp>

namespace reco {
namespace stereo_workbench {

matcher_qt_wrapper_sgbm::matcher_qt_wrapper_sgbm() :
		matcher_qt_wrapper(cv::StereoSGBM::create(0, 256, 3, 216, 864, -1, 48, 0, 0, 0,
						cv::StereoSGBM::MODE_SGBM)) {
	this->panel = new tuning_panel_sgbm(*this);
}

matcher_qt_wrapper_sgbm::~matcher_qt_wrapper_sgbm() {

}


matcher_qt_wrapper_sgbm::tuning_panel_sgbm::tuning_panel_sgbm(
		const matcher_qt_wrapper_sgbm& matcher,
		QWidget* parent)
	:tuning_panel(parent){
	construct_specialized_controls();
	connect_specialized_controls(matcher);
}


void matcher_qt_wrapper_sgbm::tuning_panel_sgbm::connect_specialized_controls(const matcher_qt_wrapper_sgbm& wrapper){
	p1_slider->                     setValue(wrapper.get_p1());
	p2_slider->                     setValue(wrapper.get_p2());
	pre_filter_cap_slider->         setValue(wrapper.get_pre_filter_cap());
	uniqueness_ratio_slider->       setValue(wrapper.get_uniqueness_ratio());

	p1_spin_box->                   setValue(wrapper.get_p1());
	p2_spin_box->                   setValue(wrapper.get_p2());
	pre_filter_cap_spin_box->       setValue(wrapper.get_pre_filter_cap());
	uniqueness_ratio_spin_box->     setValue(wrapper.get_uniqueness_ratio());

	connect(p1_slider, SIGNAL(valueChanged(int)), &wrapper, SLOT(set_p1(int)));
	connect(p2_slider, SIGNAL(valueChanged(int)), &wrapper, SLOT(set_p2(int)));
	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), &wrapper, SLOT(set_pre_filter_cap(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &wrapper, SLOT(set_uniqueness_ratio(int)));
}

void matcher_qt_wrapper_sgbm::tuning_panel_sgbm::construct_specialized_controls(){
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
	pre_filter_cap_horizontal_layout->setObjectName(QStringLiteral("pre_filter_cap_horizontal_layout"));
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

	construct_integer_control_set(
		uniqueness_ratio_horizontal_layout,
		uniqueness_ratio_label,
		uniqueness_ratio_spin_box,
		uniqueness_ratio_slider,
		tr("uniqueness ratio:"),
		QStringLiteral("uniqueness_ratio_horizontal_layout"),
		QStringLiteral("uniqueness_ratio_label"),
		QStringLiteral("uniqueness_ratio_spin_box"),
		QStringLiteral("uniqueness_ratio_slider"),
		0,256
		);
}

matcher_qt_wrapper_sgbm::tuning_panel_sgbm::~tuning_panel_sgbm(){

}

//===============================PARAMETER GETTERS==================================================
int matcher_qt_wrapper_sgbm::get_p1() const{
	return stereo_matcher->getP1();
}
int matcher_qt_wrapper_sgbm::get_p2() const{
	return stereo_matcher->getP2();
}
int matcher_qt_wrapper_sgbm::get_pre_filter_cap() const{
	return stereo_matcher->getPreFilterCap();
}
int matcher_qt_wrapper_sgbm::get_uniqueness_ratio() const{
	return stereo_matcher->getUniquenessRatio();
}

//===============================PARAMETER SETTER SLOTS=============================================
void matcher_qt_wrapper_sgbm::set_p1(int value) {
	if (value < stereo_matcher->getP2()) {
		stereo_matcher->setP1(value);
		emit parameters_changed();
	}
}

void matcher_qt_wrapper_sgbm::set_p2(int value) {
	if (value > stereo_matcher->getP1()) {
		stereo_matcher->setP2(value);
		emit parameters_changed();
	}
}

void matcher_qt_wrapper_sgbm::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	emit parameters_changed();
}

void matcher_qt_wrapper_sgbm::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	emit parameters_changed();
}

} /* namespace stereo_workbench */
} /* namespace reco */

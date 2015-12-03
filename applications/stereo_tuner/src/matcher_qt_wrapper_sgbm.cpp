/*
 * stereo_processor_sgbm.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_tuner/matcher_qt_wrapper_sgbm.hpp>

namespace reco {
namespace stereo_tuner {

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


void matcher_qt_wrapper_sgbm::tuning_panel_sgbm::connect_specialized_controls(const matcher_qt_wrapper_sgbm& matcher){
	p1_slider->                     setValue(matcher.get_p1());
	p2_slider->                     setValue(matcher.get_p2());
	pre_filter_cap_slider->         setValue(matcher.get_pre_filter_cap());
	uniqueness_ratio_slider->       setValue(matcher.get_uniqueness_ratio());
	block_size_slider->             setValue(matcher.get_bock_size());

	p1_spin_box->                   setValue(matcher.get_p1());
	p2_spin_box->                   setValue(matcher.get_p2());
	pre_filter_cap_spin_box->       setValue(matcher.get_pre_filter_cap());
	uniqueness_ratio_spin_box->     setValue(matcher.get_uniqueness_ratio());\
	block_size_spin_box->           setValue(matcher.get_bock_size());

	connect(block_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_block_size(int)));
	connect(p1_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_p1(int)));
	connect(p2_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_p2(int)));
	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_pre_filter_cap(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_uniqueness_ratio(int)));
}

void matcher_qt_wrapper_sgbm::tuning_panel_sgbm::construct_specialized_controls(){
	construct_integer_control_set(
		block_size_horizontal_layout,
		block_size_label,
		block_size_spin_box,
		block_size_slider,
		tr("block size:"),
		QStringLiteral("block_size_horizontal_layout"),
		QStringLiteral("block_size_label"),
		QStringLiteral("block_size_spin_box"),
		QStringLiteral("block_size_slider"),
		1,65,2,1
		);

	construct_integer_control_set(
		p1_horizontal_layout,
		p1_label,
		p1_spin_box,
		p1_slider,
		tr("p1:"),
		QStringLiteral("p1_horizontal_layout"),
		QStringLiteral("p1_label"),
		QStringLiteral("p1_spin_box"),
		QStringLiteral("p1_slider"),
		0,2904, 12, 0, 480
		);

	construct_integer_control_set(
		p2_horizontal_layout,
		p2_label,
		p2_spin_box,
		p2_slider,
		tr("p2:"),
		QStringLiteral("p2_horizontal_layout"),
		QStringLiteral("p2_label"),
		QStringLiteral("p2_spin_box"),
		QStringLiteral("p2_slider"),
		0,11616, 24, 0, 960
		);

	construct_integer_control_set(
		pre_filter_cap_horizontal_layout,
		pre_filter_cap_label,
		pre_filter_cap_spin_box,
		pre_filter_cap_slider,
		tr("pre-filter cap:"),
		QStringLiteral("pre_filter_cap_horizontal_layout"),
		QStringLiteral("pre_filter_cap_label"),
		QStringLiteral("pre_filter_cap_spin_box"),
		QStringLiteral("pre_filter_cap_slider"),
		0,256
		);

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

} /* namespace stereo_tuner */
} /* namespace reco */

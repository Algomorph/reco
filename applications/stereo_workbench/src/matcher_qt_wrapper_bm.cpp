/*
 * stereo_processor_bm.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/matcher_qt_wrapper_bm.hpp>
#include <QApplication>

namespace reco {
namespace stereo_workbench {


matcher_qt_wrapper_bm::matcher_qt_wrapper_bm():
		matcher_qt_wrapper(cv::StereoBM::create(256, 3)){
	this->panel = new tuning_panel_bm(*this);

}
matcher_qt_wrapper_bm::~matcher_qt_wrapper_bm(){

}


matcher_qt_wrapper_bm::tuning_panel_bm::tuning_panel_bm(
		const matcher_qt_wrapper_bm& matcher,
		QWidget* parent)
	:tuning_panel(parent){
	construct_specialized_controls();
	connect_specialized_controls(matcher);
}


void matcher_qt_wrapper_bm::tuning_panel_bm::connect_specialized_controls(const matcher_qt_wrapper_bm& matcher){
	pre_filter_type_combo_box->setCurrentIndex(matcher.get_pre_filter_type());

	pre_filter_cap_slider->         setValue(matcher.get_pre_filter_cap());
	pre_filter_size_slider->        setValue(matcher.get_pre_filter_size());
	smaller_block_size_slider->     setValue(matcher.get_smaller_block_size());
	texture_threshold_slider->      setValue(matcher.get_texture_threshold());
	uniqueness_ratio_slider->       setValue(matcher.get_uniqueness_ratio());

	pre_filter_cap_spin_box->       setValue(matcher.get_pre_filter_cap());
	pre_filter_size_spin_box->      setValue(matcher.get_pre_filter_size());
	smaller_block_size_spin_box->   setValue(matcher.get_smaller_block_size());
	texture_threshold_spin_box->    setValue(matcher.get_texture_threshold());
	uniqueness_ratio_spin_box->     setValue(matcher.get_uniqueness_ratio());

	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_pre_filter_cap(int)));
	connect(pre_filter_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_pre_filter_size(int)));
	connect(smaller_block_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_block_size(int)));
	connect(texture_threshold_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_texture_threshold(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_uniqueness_ratio(int)));
	connect(pre_filter_type_combo_box, SIGNAL(currentIndexChanged(int)),&matcher, SLOT(set_pre_filter_type(int)));
}

void matcher_qt_wrapper_bm::tuning_panel_bm::construct_specialized_controls(){

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
		pre_filter_size_horizontal_layout,
		pre_filter_size_label,
		pre_filter_size_spin_box,
		pre_filter_size_slider,
		tr("pre-filter size:"),
		QStringLiteral("pre_filter_size_horizontal_layout"),
		QStringLiteral("pre_filter_size_label"),
		QStringLiteral("pre_filter_size_spin_box"),
		QStringLiteral("pre_filter_size_slider"),
		0,256
		);


	pre_filter_type_combo_box = new QComboBox(this);
	pre_filter_type_combo_box->setObjectName(QStringLiteral("pre_filter_type_combo_box"));
	tuning_controls_vlayout->addWidget(pre_filter_type_combo_box);
	pre_filter_type_combo_box->clear();
	pre_filter_type_combo_box->insertItems(0, QStringList()
	 << QApplication::translate("main_window", "normalized response", 0)
	 << QApplication::translate("main_window", "x-sobel", 0)
	);

	construct_integer_control_set(
		smaller_block_size_horizontal_layout,
		smaller_block_size_label,
		smaller_block_size_spin_box,
		smaller_block_size_slider,
		tr("smaller block size:"),
		QStringLiteral("smaller_block_size_horizontal_layout"),
		QStringLiteral("smaller_block_size_label"),
		QStringLiteral("smaller_block_size_spin_box"),
		QStringLiteral("smaller_block_size_slider"),
		1,11,2,1
		);

	construct_integer_control_set(
		texture_threshold_horizontal_layout,
		texture_threshold_label,
		texture_threshold_spin_box,
		texture_threshold_slider,
		tr("texture threshold:"),
		QStringLiteral("texture_threshold_horizontal_layout"),
		QStringLiteral("texture_threshold_label"),
		QStringLiteral("texture_threshold_spin_box"),
		QStringLiteral("texture_threshold_slider"),
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

matcher_qt_wrapper_bm::tuning_panel_bm::~tuning_panel_bm(){

}

int matcher_qt_wrapper_bm::get_pre_filter_cap() const{
	return stereo_matcher->getPreFilterCap();
}

int matcher_qt_wrapper_bm::get_pre_filter_size() const{
	return stereo_matcher->getPreFilterSize();
}
int matcher_qt_wrapper_bm::get_pre_filter_type() const{
	return stereo_matcher->getPreFilterType();
}
int matcher_qt_wrapper_bm::get_smaller_block_size() const{
	return stereo_matcher->getSmallerBlockSize();
}
int matcher_qt_wrapper_bm::get_texture_threshold() const{
	return stereo_matcher->getTextureThreshold();
}

int matcher_qt_wrapper_bm::get_uniqueness_ratio() const{
	return stereo_matcher->getUniquenessRatio();
}

void matcher_qt_wrapper_bm::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	emit parameters_changed();
}

void matcher_qt_wrapper_bm::set_pre_filter_size(int value){
	stereo_matcher->setPreFilterSize(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_pre_filter_type(int value){
	stereo_matcher->setPreFilterType(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_smaller_block_size(int value){
	stereo_matcher->setSmallerBlockSize(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_texture_threshold(int value){
	stereo_matcher->setTextureThreshold(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bm::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	emit parameters_changed();
}





} /* namespace stereo_workbench */
} /* namespace reco */

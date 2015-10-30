/*
 * stereo_processor_bm.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/matcher_qt_wrapper_bp.hpp>
#include <QApplication>
#include <reco/utils/cpp_exception_util.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudastereo.hpp>

namespace reco {
namespace stereo_workbench {


matcher_qt_wrapper_bp::matcher_qt_wrapper_bp():
		matcher_qt_wrapper(cv::cuda::createStereoBeliefPropagation()){
	this->panel = new tuning_panel_bp(*this);

}
matcher_qt_wrapper_bp::~matcher_qt_wrapper_bp(){

}


matcher_qt_wrapper_bp::tuning_panel_bp::tuning_panel_bp(
		const matcher_qt_wrapper_bp& matcher,
		QWidget* parent)
	:tuning_panel(parent){
	construct_specialized_controls();
	connect_specialized_controls(matcher);
}


void matcher_qt_wrapper_bp::tuning_panel_bp::connect_specialized_controls(const matcher_qt_wrapper_bp& matcher){
	pre_filter_type_combo_box->setCurrentIndex(matcher.get_pre_filter_type());

	block_size_slider->             setValue(matcher.get_bock_size());
	pre_filter_cap_slider->         setValue(matcher.get_pre_filter_cap());
	pre_filter_size_slider->        setValue(matcher.get_pre_filter_size());
	smaller_block_size_slider->     setValue(matcher.get_smaller_block_size());
	texture_threshold_slider->      setValue(matcher.get_texture_threshold());
	uniqueness_ratio_slider->       setValue(matcher.get_uniqueness_ratio());

	block_size_spin_box->           setValue(matcher.get_bock_size());
	pre_filter_cap_spin_box->       setValue(matcher.get_pre_filter_cap());
	pre_filter_size_spin_box->      setValue(matcher.get_pre_filter_size());
	smaller_block_size_spin_box->   setValue(matcher.get_smaller_block_size());
	texture_threshold_spin_box->    setValue(matcher.get_texture_threshold());
	uniqueness_ratio_spin_box->     setValue(matcher.get_uniqueness_ratio());

	connect(block_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_block_size(int)));
	connect(pre_filter_cap_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_pre_filter_cap(int)));
	connect(pre_filter_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_pre_filter_size(int)));
	connect(smaller_block_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_block_size(int)));
	connect(texture_threshold_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_texture_threshold(int)));
	connect(uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_uniqueness_ratio(int)));
	connect(pre_filter_type_combo_box, SIGNAL(currentIndexChanged(int)),&matcher, SLOT(set_pre_filter_type(int)));
}

void matcher_qt_wrapper_bp::tuning_panel_bp::construct_specialized_controls(){
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
			5,65,2,1
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

void matcher_qt_wrapper_bp::compute(const cv::Mat& left,const cv::Mat& right, cv::Mat& disparity){
	if(left.type() == CV_8UC3 && right.type() == CV_8UC3){
		cv::Mat left_target,right_target;
		cv::cvtColor(left,left_target,cv::COLOR_BGR2GRAY);
		cv::cvtColor(right,right_target,cv::COLOR_BGR2GRAY);
		this->stereo_matcher->compute(left_target,right_target,disparity);
	}else if(left.type() == CV_8UC1 && right.type() == CV_8UC1){
		this->stereo_matcher->compute(left,right,disparity);
	}else{
		err2(std::invalid_argument,"Expecting both left & right matrices both to have type CV_8UC3 or CV_8UC1.");
	}
}

matcher_qt_wrapper_bp::tuning_panel_bp::~tuning_panel_bp(){

}

double matcher_qt_wrapper_bp::get_data_weight() const{
	return stereo_matcher->getDataWeight();
}

double matcher_qt_wrapper_bp::get_disc_single_jump() const{
	return stereo_matcher->getDiscSingleJump();
}
double matcher_qt_wrapper_bp::get_max_data_term() const{
	return stereo_matcher->getMaxDataTerm();
}
double matcher_qt_wrapper_bp::get_max_disc_term() const{
	return stereo_matcher->getMaxDiscTerm();
}
int matcher_qt_wrapper_bp::get_msg_type() const{
	return stereo_matcher->getMsgType();
}

int matcher_qt_wrapper_bp::get_num_iters() const{
	return stereo_matcher->getNumIters();
}

int matcher_qt_wrapper_bp::get_num_levels() const{
	return stereo_matcher->getNumLevels();
}

void matcher_qt_wrapper_bp::set_data_weight(double value) {
	stereo_matcher->setDataWeight(value);
	emit parameters_changed();
}

void matcher_qt_wrapper_bp::set_disc_single_jump(double value){
	stereo_matcher->setDiscSingleJump(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bp::set_max_data_term(double value){
	stereo_matcher->setMaxDataTerm(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bp::set_max_disc_term(double value){
	stereo_matcher->setMaxDiscTerm(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bp::set_msg_type(int value){
	stereo_matcher->setMsgType(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bp::set_num_iters(int value) {
	stereo_matcher->setNumIters(value);
	emit parameters_changed();
}
void matcher_qt_wrapper_bp::set_num_levels(int value) {
	stereo_matcher->setNumLevels(value);
	emit parameters_changed();
}





} /* namespace stereo_workbench */
} /* namespace reco */

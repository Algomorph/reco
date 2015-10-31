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
	msg_type_combo_box->setCurrentIndex(matcher.get_msg_type() == CV_16SC1 ? 0 : 1);

	block_size_slider->             setValue(matcher.get_bock_size());
	num_iters_slider->      		setValue(matcher.get_num_iters());
	num_levels_slider->      		setValue(matcher.get_num_levels());

	block_size_spin_box->           setValue(matcher.get_bock_size());
	data_weight_spin_box->			setValue(matcher.get_data_weight());
	disc_single_jump_spin_box-> 	setValue(matcher.get_disc_single_jump());
	max_data_term_spin_box->		setValue(matcher.get_max_data_term());
	max_disc_term_spin_box->		setValue(matcher.get_max_disc_term());
	num_iters_spin_box->      		setValue(matcher.get_num_iters());
	num_levels_spin_box->     		setValue(matcher.get_num_levels());

	connect(block_size_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_block_size(int)));

	connect(data_weight_spin_box, SIGNAL(valueChanged(double)), &matcher, SLOT(set_data_weight(double)));
	connect(disc_single_jump_spin_box, SIGNAL(valueChanged(double)), &matcher, SLOT(set_disc_single_jump(double)));
	connect(max_data_term_spin_box, SIGNAL(valueChanged(double)), &matcher, SLOT(set_max_data_term(double)));
	connect(max_disc_term_spin_box, SIGNAL(valueChanged(double)), &matcher, SLOT(set_max_disc_term(double)));

	connect(num_iters_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_num_iters(int)));
	connect(num_levels_slider, SIGNAL(valueChanged(int)), &matcher, SLOT(set_num_levels(int)));
	connect(msg_type_combo_box, SIGNAL(currentIndexChanged(int)),&matcher, SLOT(set_msg_type(int)));
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

	construct_double_control_set(
		data_weight_horizontal_layout,
		data_weight_label,
		data_weight_spin_box,
		tr("data weight:"),
		QStringLiteral("data_weight_horizontal_layout"),
		QStringLiteral("data_weight_label"),
		QStringLiteral("data_weight_spin_box"),
		0.,256.
		);

	construct_double_control_set(
		disc_single_jump_horizontal_layout,
		disc_single_jump_label,
		disc_single_jump_spin_box,
		tr("disc single jump:"),
		QStringLiteral("disc_single_jump_horizontal_layout"),
		QStringLiteral("disc_single_jump_label"),
		QStringLiteral("disc_single_jump_spin_box"),
		0.,256.
		);

	construct_double_control_set(
		max_data_term_horizontal_layout,
		max_data_term_label,
		max_data_term_spin_box,
		tr("disc single jump:"),
		QStringLiteral("max_data_term_horizontal_layout"),
		QStringLiteral("max_data_term_label"),
		QStringLiteral("max_data_term_spin_box"),
		0.,256.
		);

	construct_double_control_set(
		max_disc_term_horizontal_layout,
		max_disc_term_label,
		max_disc_term_spin_box,
		tr("disk single jump:"),
		QStringLiteral("max_disc_term_horizontal_layout"),
		QStringLiteral("max_disc_term_label"),
		QStringLiteral("max_disc_term_spin_box"),
		0.,256.
		);

	msg_type_combo_box = new QComboBox(this);
	msg_type_combo_box->setObjectName(QStringLiteral("msg_type_combo_box"));
	tuning_controls_vlayout->addWidget(msg_type_combo_box);
	msg_type_combo_box->clear();

	//indexes are currently hard-coded
	msg_type_combo_box->insertItems(0, QStringList()
	 << QApplication::translate("main_window", "16-bit", 0)
	 << QApplication::translate("main_window", "32-bit", 0)
	);


	construct_integer_control_set(
		num_iters_horizontal_layout,
		num_iters_label,
		num_iters_spin_box,
		num_iters_slider,
		tr("num. iterations/level:"),
		QStringLiteral("num_iters_horizontal_layout"),
		QStringLiteral("num_iters_label"),
		QStringLiteral("num_iters_spin_box"),
		QStringLiteral("num_iters_slider"),
		0,256
		);

	construct_integer_control_set(
		num_levels_horizontal_layout,
		num_levels_label,
		num_levels_spin_box,
		num_levels_slider,
		tr("num. levels:"),
		QStringLiteral("num_levels_horizontal_layout"),
		QStringLiteral("num_levels_label"),
		QStringLiteral("num_levels_spin_box"),
		QStringLiteral("num_levels_slider"),
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
void matcher_qt_wrapper_bp::set_msg_type(int combo_box_index){
	stereo_matcher->setMsgType(combo_box_index == 0? CV_16SC1 : CV_32FC1);
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

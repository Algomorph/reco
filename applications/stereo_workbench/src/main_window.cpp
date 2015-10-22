/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
#include "../ui_main_window.h"
#include "main_window.h"
#include "calibu_rectifier.h"
#include "opencv_rectifier.h"

//datapipe
#include <reco/datapipe/hal_stereo_pipe.h>
#include <reco/datapipe/hal_interop.h>

//utils
#include <reco/utils/swap_buffer.h>
#include <reco/utils/debug_util.h>

//qt
#include <QMessageBox>
#include <QFileDialog>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace reco {
namespace stereo_workbench {

#define DEFAULT_RECO_DATA_PATH "/media/algomorph/Data/reco/"
#define DEFAULT_CAP_PATH DEFAULT_RECO_DATA_PATH "cap/yi/"
#define DEFAULT_CALIB_PATH "/home/algomorph/Dropbox/calib/yi/"

#define VIDEO_LEFT "sm01l_edit.mp4"
#define VIDEO_RIGHT "sm01r_edit.mp4"
#define CALIB_FILE "s25cv12_BEST.xml"

main_window::main_window() :
		ui(new Ui_main_window),
		video_buffer(new utils::optimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		pipe(),
		//stereo_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>()),
		stereo_input_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		stereo_output_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		tuner(stereo_input_buffer,stereo_output_buffer)
{
	using namespace boost::filesystem;
	ui->setupUi(this);
	ui->disparity_viewer->configure_for_pipe(1);

	//check default files
	path calib_path(DEFAULT_CALIB_PATH CALIB_FILE);
	if(is_regular_file(calib_path)){
		tuner.set_rectifier(std::shared_ptr<rectifier>(new opencv_rectifier(calib_path.c_str())));
	}

	if(is_regular_file(path(DEFAULT_CAP_PATH VIDEO_LEFT)) && is_regular_file(path(DEFAULT_CAP_PATH VIDEO_RIGHT))){
		pipe.reset(
		new datapipe::hal_stereo_pipe(video_buffer,datapipe::hal_stereo_pipe::video_files,{
							DEFAULT_CAP_PATH VIDEO_LEFT,
							DEFAULT_CAP_PATH VIDEO_RIGHT
						}
		#ifdef UNDISTORT_ON_CAPTURE
				,"/media/algomorph/Data/reco/calib/yi/" CALIB_FILE
		#endif
				));
	}

	connect_actions();

	hook_pipe();
	tuner.run();
	connect(&tuner,SIGNAL(frame(std::shared_ptr<std::vector<cv::Mat>>)),
			ui->disparity_viewer,SLOT(on_frame(std::shared_ptr<std::vector<cv::Mat>>)));
}

main_window::~main_window() {
	delete ui;
}


/**
 * Connect actions of menus & signals of controls with the corresponding slot functions
 */
void main_window::connect_actions() {

//====================== ACTIONS ===================================================================
	connect(ui->action_open_calibration_file,SIGNAL(triggered()),this,SLOT(open_calibration_file()));
	connect(ui->action_open_image_pair,SIGNAL(triggered()),this,SLOT(open_image_pair()));
//==================================================================================================
	ui->rectify_checkbox->setChecked(tuner.is_rectification_enabled());
//====================== SLIDER / SPINNER CONTROLS =================================================

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)
	ui->minimum_disparity_slider->setValue(tuner.stereo_matcher.minDisparity);
	ui->number_of_disparities_slider->setValue(tuner.stereo_matcher.numberOfDisparities);
	ui->window_size_slider->setValue(tuner.stereo_matcher.SADWindowSize);
	ui->p1_slider->setValue(tuner.stereo_matcher.P1);
	ui->p2_slider->setValue(tuner.stereo_matcher.P2);
	ui->pre_filter_cap_slider->setValue(tuner.stereo_matcher.preFilterCap);
	ui->uniqueness_ratio_slider->setValue(tuner.stereo_matcher.uniquenessRatio);
	ui->speckle_window_size_slider->setValue(tuner.stereo_matcher.speckleWindowSize);
	ui->speckle_range_slider->setValue(tuner.stereo_matcher.speckleRange);

	ui->minimum_disparity_spin_box->setValue(tuner.stereo_matcher.minDisparity);
	ui->number_of_disparities_spin_box->setValue(tuner.stereo_matcher.numberOfDisparities);
	ui->window_size_spin_box->setValue(tuner.stereo_matcher.SADWindowSize);
	ui->p1_spin_box->setValue(tuner.stereo_matcher.P1);
	ui->p2_spin_box->setValue(tuner.stereo_matcher.P2);
	ui->pre_filter_cap_spin_box->setValue(tuner.stereo_matcher.preFilterCap);
	ui->uniqueness_ratio_spin_box->setValue(tuner.stereo_matcher.uniquenessRatio);
	ui->speckle_window_size_spin_box->setValue(tuner.stereo_matcher.speckleWindowSize);
	ui->speckle_range_spin_box->setValue(tuner.stereo_matcher.speckleRange);
#elif CV_VERSION_MAJOR == 3
	ui->minimum_disparity_slider->      setValue(tuner.stereo_matcher->getMinDisparity());
	ui->number_of_disparities_slider->  setValue(tuner.stereo_matcher->getNumDisparities());
	ui->window_size_slider->            setValue(tuner.stereo_matcher->getBlockSize());
	ui->p1_slider->                     setValue(tuner.stereo_matcher->getP1());
	ui->p2_slider->                     setValue(tuner.stereo_matcher->getP2());
	ui->pre_filter_cap_slider->         setValue(tuner.stereo_matcher->getPreFilterCap());
	ui->uniqueness_ratio_slider->       setValue(tuner.stereo_matcher->getUniquenessRatio());
	ui->speckle_window_size_slider->    setValue(tuner.stereo_matcher->getSpeckleWindowSize());
	ui->speckle_range_slider->          setValue(tuner.stereo_matcher->getSpeckleRange());
	ui->v_offset_slider->               setValue(tuner.get_v_offset());

	ui->minimum_disparity_spin_box->    setValue(tuner.stereo_matcher->getMinDisparity());
	ui->number_of_disparities_spin_box->setValue(tuner.stereo_matcher->getNumDisparities());
	ui->window_size_spin_box->          setValue(tuner.stereo_matcher->getBlockSize());
	ui->p1_spin_box->                   setValue(tuner.stereo_matcher->getP1());
	ui->p2_spin_box->                   setValue(tuner.stereo_matcher->getP2());
	ui->pre_filter_cap_spin_box->       setValue(tuner.stereo_matcher->getPreFilterCap());
	ui->uniqueness_ratio_spin_box->     setValue(tuner.stereo_matcher->getUniquenessRatio());
	ui->speckle_window_size_spin_box->  setValue(tuner.stereo_matcher->getSpeckleWindowSize());
	ui->speckle_range_spin_box->        setValue(tuner.stereo_matcher->getSpeckleRange());
	ui->v_offset_spin_box->             setValue(tuner.get_v_offset());
#endif
	connect(ui->minimum_disparity_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_minimum_disparity(int)));
	connect(ui->minimum_disparity_slider, SIGNAL(valueChanged(int)), ui->minimum_disparity_spin_box, SLOT(setValue(int)));
	connect(ui->minimum_disparity_spin_box, SIGNAL(valueChanged(int)), ui->minimum_disparity_slider, SLOT(setValue(int)));
	connect(ui->number_of_disparities_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_num_disparities(int)));
	connect(ui->number_of_disparities_slider, SIGNAL(valueChanged(int)), ui->number_of_disparities_spin_box, SLOT(setValue(int)));
	connect(ui->number_of_disparities_spin_box, SIGNAL(valueChanged(int)), ui->number_of_disparities_slider, SLOT(setValue(int)));
	connect(ui->window_size_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_window_size(int)));
	connect(ui->window_size_slider, SIGNAL(valueChanged(int)), ui->window_size_spin_box, SLOT(setValue(int)));
	connect(ui->window_size_spin_box, SIGNAL(valueChanged(int)), ui->window_size_slider, SLOT(setValue(int)));
	connect(ui->p1_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_p1(int)));
	connect(ui->p1_slider, SIGNAL(valueChanged(int)), ui->p1_spin_box, SLOT(setValue(int)));
	connect(ui->p1_spin_box, SIGNAL(valueChanged(int)), ui->p1_slider, SLOT(setValue(int)));
	connect(ui->p2_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_p2(int)));
	connect(ui->p2_slider, SIGNAL(valueChanged(int)), ui->p2_spin_box, SLOT(setValue(int)));
	connect(ui->p2_spin_box, SIGNAL(valueChanged(int)), ui->p2_slider, SLOT(setValue(int)));
	connect(ui->pre_filter_cap_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_pre_filter_cap(int)));
	connect(ui->pre_filter_cap_slider, SIGNAL(valueChanged(int)), ui->pre_filter_cap_spin_box, SLOT(setValue(int)));
	connect(ui->pre_filter_cap_spin_box, SIGNAL(valueChanged(int)), ui->pre_filter_cap_slider, SLOT(setValue(int)));
	connect(ui->uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_uniqueness_ratio(int)));
	connect(ui->uniqueness_ratio_slider, SIGNAL(valueChanged(int)), ui->uniqueness_ratio_spin_box, SLOT(setValue(int)));
	connect(ui->uniqueness_ratio_spin_box, SIGNAL(valueChanged(int)), ui->uniqueness_ratio_slider, SLOT(setValue(int)));
	connect(ui->speckle_window_size_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_speckle_window_size(int)));
	connect(ui->speckle_window_size_slider, SIGNAL(valueChanged(int)), ui->speckle_window_size_spin_box, SLOT(setValue(int)));
	connect(ui->speckle_window_size_spin_box, SIGNAL(valueChanged(int)), ui->speckle_window_size_slider, SLOT(setValue(int)));
	connect(ui->speckle_range_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_speckle_range(int)));
	connect(ui->speckle_range_slider, SIGNAL(valueChanged(int)), ui->speckle_range_spin_box, SLOT(setValue(int)));
	connect(ui->speckle_range_spin_box, SIGNAL(valueChanged(int)), ui->speckle_range_slider, SLOT(setValue(int)));
	connect(ui->v_offset_slider, SIGNAL(valueChanged(int)), &tuner, SLOT(set_v_offset(int)));
	connect(ui->v_offset_slider, SIGNAL(valueChanged(int)), ui->v_offset_spin_box, SLOT(setValue(int)));
	connect(ui->v_offset_spin_box, SIGNAL(valueChanged(int)), ui->v_offset_slider, SLOT(setValue(int)));

}

/**
 * Connect the pipe to output and all related buttons
 */
void main_window::hook_pipe(){
	if(pipe){
		ui->stereo_feed_viewer->configure_for_pipe(pipe->get_num_channels());
		connect(pipe.get(),SIGNAL(frame()), this, SLOT(handle_frame()));
		connect(ui->capture_button, SIGNAL(released()), pipe.get(), SLOT(run()));
		connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(pause()));
	}
}

/**
 * Disconnect existing pipe from everything and destroy it
 * TODO: 078 is this method needed?
 */
void main_window::unhook_pipe(){
	if(pipe){
		disconnect(ui->capture_button,0,0,0);
		disconnect(ui->pause_button,0,0,0);
	}
}

/**
 * Triggered on each frame emergent from the pipe
 */
void main_window::handle_frame(){
	std::shared_ptr<hal::ImageArray> images = video_buffer->pop_front();
	if(images){
		stereo_input_buffer->push_back(images);
		ui->stereo_feed_viewer->on_frame(images);
	}else{
		std::shared_ptr<hal::ImageArray> dummy;
		video_buffer->clear();
		video_buffer->push_back(dummy);//send another dummy to signal end
	}

}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {
	pipe->stop();
	//halt frame consumption
	std::shared_ptr<hal::ImageArray> dummy;
	video_buffer->clear();
	video_buffer->push_back(dummy);//send dummy to signal end
	this->tuner.stop();

}

void main_window::open_image_pair(){
	QFileDialog dialog(this,tr("Open Image Files"), DEFAULT_CAP_PATH, tr("Image files (*.jpg *.png *.bmp)"));
	dialog.setFileMode(QFileDialog::ExistingFiles);
	QStringList file_names;
	if(dialog.exec()){
		file_names = dialog.selectedFiles();
	}
	file_names.sort();
	if(file_names.empty())
		return;

	if(file_names.size() != 2){
		QMessageBox msg_box;
		msg_box.setText("Expecting exactly two files, a right-left image pair. Got: " + QString::number(file_names.size()));
		msg_box.exec();
		return;
	}

	cv::Mat left = cv::imread(file_names[0].toStdString());
	cv::Mat right = cv::imread(file_names[1].toStdString());

	std::vector<cv::Mat> matrices = {left,right};
	std::shared_ptr<hal::ImageArray> images = datapipe::hal_array_from_cv(matrices);
	pipe->pause();//pause to avoid conflicts
	ui->stereo_feed_viewer->on_frame(images);
	stereo_input_buffer->push_back(images);

}

/**
 * Loads calibration file from XML to enable rectification
 */
void main_window::open_calibration_file(){
	QString qfile_path = QFileDialog::getOpenFileName(this, tr("Open Calibration File"),
			DEFAULT_CALIB_PATH, tr("Calibration files (*.xml)"));

	if (!qfile_path.isEmpty()) {
		std::string file_path = qfile_path.toStdString();
		tuner.set_rectifier(std::shared_ptr<rectifier>(new calibu_rectifier(calibu::ReadXmlRig(file_path))));
	}
}

/**
 * Saves the currently processed stereo image pair as "left.png" and "right.png"
 */
void main_window::on_save_current_button_clicked(){
	tuner.save_current();
}

/**
 * Enables or disables stereo rectification
 */
void main_window::on_rectify_checkbox_clicked(){
	tuner.toggle_rectification();
}



} //end namespace reco
} //end namespace stereo_workbench


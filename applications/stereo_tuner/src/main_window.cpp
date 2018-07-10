/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
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
#include <reco/stereo/calibu_rectifier.hpp>
#include <reco/stereo/opencv_rectifier.hpp>
#include <reco/stereo_tuner/matcher_qt_wrapper.hpp>
#include <reco/stereo_tuner/matcher_qt_wrapper_sgbm.hpp>
#include <src/main_window.h>
#include "ui_main_window.h"

namespace reco {
namespace stereo_tuner {

#define DEFAULT_RECO_DATA_PATH "/media/algomorph/Data/reco/"
#define DEFAULT_CAP_PATH DEFAULT_RECO_DATA_PATH "cap/"
#define DEFAULT_CALIB_PATH "/home/algomorph/Dropbox/calib/yi/"

//uncomment to load images instead of videos by default
#define DEFAULT_LOAD_IMAGES

#define DEFAULT_VIDEO_LEFT DEFAULT_CAP_PATH "0_1_calib01/1_sample_edit.mp4"
#define DEFAULT_VIDEO_RIGHT DEFAULT_CAP_PATH "0_1_calib01/0_sample_edit.mp4"

#define CALIB_FILE "0_1_smallboard_redux.xml"


#define DEFAULT_IMAGE_LEFT DEFAULT_CAP_PATH "mouse_cap03/test_pairs/dl_18_rect.pnga"
#define DEFAULT_IMAGE_RIGHT DEFAULT_CAP_PATH "mouse_cap03/test_pairs/dr_18_rect.pnga"

main_window::main_window() :
		ui(new Ui_main_window),
		video_buffer(new utils::optimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		pipe(),
		//stereo_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::Imag	eArray>>()),
		stereo_input_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		stereo_output_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		processor(stereo_input_buffer,stereo_output_buffer)
{
	using namespace boost::filesystem;
	ui->setupUi(this);
	//TODO: these "configure_for_pipe" functions should be renamed to something more comprehensible
	ui->stereo_feed_viewer->set_channel_number(2);
	ui->disparity_viewer->set_channel_number(1);
	ui->disparity_viewer->set_blank(1920,1080);

	//----------SET UP STEREO PROCESSING
	//check default files
	path calib_path(DEFAULT_CALIB_PATH CALIB_FILE);
	if(is_regular_file(calib_path)){
		processor.set_rectifier(std::shared_ptr<stereo::rectifier>(new stereo::opencv_rectifier(calib_path.c_str())));
		processor.toggle_rectification();
	}


	processor.set_matcher(ui->tuner_panel->matchers[stereo_matcher_tuning_panel::sgbm]);
	connect(&processor,SIGNAL(frame(std::shared_ptr<std::vector<cv::Mat>>)),
				ui->disparity_viewer,SLOT(on_frame(std::shared_ptr<std::vector<cv::Mat>>)));
	//TODO: processor should be a member of ui->tuner_panel from the get-go
	ui->tuner_panel->connect_stereo_processor(this->processor);
	processor.run();
	connect_actions();

	//---------LOAD INPUT-------------
#ifdef DEFAULT_LOAD_IMAGES
	if(is_regular_file(path(DEFAULT_IMAGE_LEFT)) && is_regular_file(path(DEFAULT_IMAGE_RIGHT))){

		cv::Mat left = cv::imread(DEFAULT_IMAGE_LEFT);
		cv::Mat right = cv::imread(DEFAULT_IMAGE_RIGHT);

		std::vector<cv::Mat> matrices = {left,right};
		std::shared_ptr<hal::ImageArray> images = datapipe::hal_array_from_cv(matrices);
		stereo_input_buffer->push_back(images);
		ui->stereo_feed_viewer->on_frame(images);
	}
#else
	if(is_regular_file(path(DEFAULT_VIDEO_LEFT)) && is_regular_file(path(DEFAULT_VIDEO_RIGHT))){
		pipe.reset(
		new datapipe::hal_stereo_pipe(video_buffer,datapipe::hal_stereo_pipe::video_files,{
							DEFAULT_VIDEO_LEFT,
							DEFAULT_VIDEO_RIGHT
						}
		#ifdef UNDISTORT_ON_CAPTURE
				,"/media/algomorph/Data/reco/calib/yi/" CALIB_FILE
		#endif
				));
	}
#endif

	hook_pipe();

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


}

/**
 * Connect the pipe to output and all related buttons
 */
void main_window::hook_pipe(){
	if(pipe){
		ui->stereo_feed_viewer->set_channel_number(pipe->get_num_channels());
		connect(pipe.get(),SIGNAL(frame()), this, SLOT(handle_frame()));
		connect(ui->capture_button, SIGNAL(released()), pipe.get(), SLOT(run()));
		connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(pause()));
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
	if(pipe){
		pipe->stop();
		//halt frame consumption
		std::shared_ptr<hal::ImageArray> dummy;
		video_buffer->clear();
		video_buffer->push_back(dummy);//send dummy to signal end
	}
	this->processor.stop();

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
	if(pipe){
		pipe->pause();//pause to avoid conflicts
	}
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
		processor.set_rectifier(std::shared_ptr<stereo::rectifier>(new stereo::opencv_rectifier(calibu::ReadXmlRig(file_path))));
	}
}

/**
 * Saves the currently processed stereo image pair as "left.png" and "right.png"
 */
void main_window::on_save_current_button_clicked(){
	processor.save_current_matcher_input();
}

/**
 * Enables or disables stereo rectification
 */
void main_window::on_rectify_checkbox_clicked(){
	processor.toggle_rectification();
}



} //end namespace reco
} //end namespace stereo_tuner


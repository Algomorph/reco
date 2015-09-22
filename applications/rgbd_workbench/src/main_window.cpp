/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
#include <reco/datapipe/kinect_v2_info.h>
#include <reco/datapipe/kinect2_pipe.h>
#include <QThread>
#include <QDebug>
#include <QFileDialog>

// Point Cloud Library


// ARPG includes
#include <calibu/Calibu.h>
#include <HAL/Camera/CameraDevice.h>

//OpenCV Includes
#include <opencv2/core/eigen.hpp>

//utils
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>
#include <reco/utils/color_util.h>
#include "main_window.h"
#include "../ui_main_window.h"

namespace reco {
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080

#define DEFAULT_RECO_DATA_PATH "/media/algomorph/Data/reco/"
#define DEFAULT_CAP_PATH DEFAULT_RECO_DATA_PATH "cap/"
#define DEFAULT_CALIB_PATH DEFAULT_RECO_DATA_PATH "calib/"

#define DEFAULT_CALIBRATION_FILE_PATH DEFAULT_CALIB_PATH "pos_D_2_kinects.xml"
#define DEFAULT_LOG_FILE_PATH DEFAULT_CAP_PATH "pos_D_slow_rotating_human_2_kinects_1240_frames.log"
//#define DEFAULT_LOG_FILE_PATH "/media/algomorph/Data/reco/cap/pos_E_moving_human_4_kinects.log"

main_window::main_window() :
		ui(new Ui_main_window),
				rgb_viewer("RGB Feed", NULL),
				depth_viewer("Depth Feed", NULL),
				pipe_buffer(new utils::optimistic_assignment_swap_buffer<
								std::shared_ptr<hal::ImageArray>>()),
				pipe(new datapipe::kinect2_pipe(pipe_buffer, datapipe::kinect2_pipe::hal_log,
								DEFAULT_LOG_FILE_PATH)),
				pipe_signals_hooked(false),
				calibration_loaded(false),
				num_frames_in_reconstruction_queue(0),
				reco_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>()),
				reco_output_buffer(new point_cloud_buffer())
{
	ui->setupUi(this);
	connect_actions();
	//start pipe with default file
	hook_pipe_signals();
	//connect signals to update the # of frames processed
	connect(reco_output_buffer.get(), SIGNAL(size_changed(size_t)), this, SLOT(update_reco_processed_label(size_t)));
	load_calibration(DEFAULT_CALIBRATION_FILE_PATH);
	/*TODO: check if this necessarily has to be done AFTER call to ui->setupUi(this).
	If not, revise down from a pointer to a simple member*/
	cloud_viewer.reset(new point_cloud_viewer(reco_output_buffer,this->ui->qvtk_widget));

}

main_window::~main_window() {
	delete ui;
}


/**
 * Connect actions of menus with the corresponding slot functions
 */

void main_window::connect_actions() {
	connect(ui->action_open_kinect_devices, SIGNAL(triggered()), this, SLOT(open_kinect_devices()));
	connect(ui->action_open_hal_log, SIGNAL(triggered()), this, SLOT(open_hal_log()));
	connect(ui->action_open_image_folder, SIGNAL(triggered()), this, SLOT(open_image_folder()));
	connect(ui->action_open_video_files, SIGNAL(triggered()), this, SLOT(open_video_files()));
	connect(ui->action_open_calibration_file, SIGNAL(triggered()),this, SLOT(open_calibration_file()));
	connect(ui->action_close_stream, SIGNAL(triggered()), this, SLOT(unhook_pipe_signals()));

}

/**
 * Open kinect feed source from actual devices (if possible)
 */
void main_window::open_kinect_devices() {
	unhook_pipe_signals();
	pipe.reset(new datapipe::kinect2_pipe(pipe_buffer, datapipe::kinect2_pipe::kinect2_device));
	hook_pipe_signals();
}
/**
 * Open kinect feed source from hal log file
 */
void main_window::open_hal_log() {
	QString file_name = QFileDialog::getOpenFileName(this, tr("Open Log File"),
			DEFAULT_CAP_PATH, tr("HAL Log files (*.log)"));

	if (!file_name.isEmpty()) {
		unhook_pipe_signals();
		//TODO: test if QString --> std::string works on windows like this
		pipe.reset(
				new datapipe::kinect2_pipe(pipe_buffer, datapipe::kinect2_pipe::hal_log,
						file_name.toStdString()));
		hook_pipe_signals();

	}
}

/**
 * Check to see if both the calibration file has been loaded and the pipe signals have been hooked,
 * if so: enable the reconstruction processing & playback controls
 */
void main_window::toggle_reco_controls() {
	if (this->calibration_loaded && this->pipe_signals_hooked) {
		this->ui->reco_proc_group->setEnabled(true);
		this->ui->reco_playback_group->setEnabled(true);
	}else{
		this->ui->reco_proc_group->setEnabled(false);
		this->ui->reco_playback_group->setEnabled(false);
	}
}


/**
 * Open kinect feed from image folder
 */
void main_window::open_image_folder() {
	throw reco::utils::not_implemented();
}

/**
 * Load the calibration from file path returned by the FileDialog
 */
void main_window::open_calibration_file() {
	QString qfile_path = QFileDialog::getOpenFileName(this, tr("Open Calibration File"),
			DEFAULT_CALIB_PATH, tr("Calibu calibration files (*.xml)"));

	if (pipe_signals_hooked && !qfile_path.isEmpty()) {
		std::string file_path = qfile_path.toStdString();

		load_calibration(file_path);
	}
}

void main_window::open_video_files(){
	QFileDialog dialog(this,tr("Open Video Files"), DEFAULT_CAP_PATH, tr("Video files (*.avi, *.mp4, *.mov)"));
	dialog.setFileMode(QFileDialog::ExistingFiles);
	QStringList file_names;
	if(dialog.exec()){
		file_names = dialog.selectedFiles();
	}
	file_names.sort();

}

/**
 * Connect pipe signals from output and GUI buttons to pipe slots
 */
void main_window::hook_pipe_signals() {
	this->unhook_pipe_signals();
	//set up error reporting;
	connect(pipe.get(), SIGNAL(error(QString)), this, SLOT(report_error(QString)));
	//connect the play and pause buttons
	connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(pause()));
	connect(ui->play_button, SIGNAL(released()), pipe.get(), SLOT(run()));
	//connect the pipe output to viewer
	connect(pipe.get(), SIGNAL(frame()), this, SLOT(on_frame()));
	rgb_viewer.configure_for_pipe(pipe->get_num_channels());
	depth_viewer.configure_for_pipe(pipe->get_num_channels());

	//colors have to be reinitialized per chance the number of kinects has changed
	pipe_signals_hooked = true; //set flag
}

/**
 * Disconnect pipe signals between output/GUI buttons and pipe slots,
 * release all resources assosiated with the pipe and shut the pipe down
 */
void main_window::unhook_pipe_signals() {
	if (pipe_signals_hooked) {
		//disconnect the viewer windows from the pipe
		disconnect(pipe.get(), 0, 0, 0);
		//disconnect the pipe from the play, pause buttons
		disconnect(ui->play_button, 0, 0, 0);
		disconnect(ui->pause_button, 0, 0, 0);
		//reset the viewers
		this->rgb_viewer.clear_gui_configuration();
		this->depth_viewer.clear_gui_configuration();
		//------
		toggle_reco_controls();
		shut_pipe_down();
		//this will stop the reconstruction and clear reconstruction output, as well as
		//disconnect the old worker's signals
		reconstruction_worker.reset();
		//unset flags
		pipe_signals_hooked = false;
		calibration_loaded = false;
	}
}

/**
 * Shut down the RGBD video source pipe permanently
 */
void main_window::shut_pipe_down() {
	pipe->stop();
	std::shared_ptr<hal::ImageArray> dummy;
	//clear in case of last frame push success
	pipe_buffer->clear();
	//push back dummy in case of trying to get from empty buffer
	pipe_buffer->push_back(dummy);
}

/**
 * Displays the current frame from the buffer on the screen, queues the current frame
 * into the reconstruction process
 */
void main_window::on_frame() {
	std::shared_ptr<hal::ImageArray> images = this->pipe_buffer->pop_front();
	//enqueue
	this->reco_input_buffer->push_back(images);
	num_frames_in_reconstruction_queue++;
	this->ui->reco_queued_label->setText(QString::number(num_frames_in_reconstruction_queue));
	//display
	this->rgb_viewer.on_frame(images);
	this->depth_viewer.on_frame(images);
}

void main_window::update_reco_processed_label(size_t value){
	ui->reco_processed_label->setText(QString::number(value));
}

/**
 * Slot for error reporting (eventually, errors emanating from child threads should print the error
 * to stdout or stderr
 * @param string - error message
 */
void main_window::report_error(QString string) {
	qDebug() << string;
}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {
	rgb_viewer.close();
	depth_viewer.close();
	unhook_pipe_signals();
}

/**
 * Load calibration (Calibu format) from the file at the given path
 * @param file_path path to the Calibu calibration file
 */
void main_window::load_calibration(std::string file_path){
	//parse intrinsics
	calibration.reset(new calibration_parameters(file_path));

	//to aviod magic numbers
	const int num_kinects = pipe->get_num_kinects();

	//check against the pipe's number of channels
	if (calibration->get_num_kinects() != pipe->get_num_kinects()) {
		err(std::invalid_argument)
				<< "The number of kinect feeds in the provided calibration file ("
				<< calibration->get_num_kinects()
				<< ") does not correspond to the number of kinect feeds in the provided log file (presumably, "
				<< num_kinects << ")." << enderr;
	}
	calibration_loaded = true;

	reconstruction_worker.reset(new reconstructor(reco_input_buffer,reco_output_buffer,calibration));
	reconstructor* reco_p = reconstruction_worker.get();
	connect(reco_p,SIGNAL(frame_consumed()),this,SLOT(decrease_queue_counter()));

	if(pipe_signals_hooked){
		toggle_reco_controls();
	}
}

void main_window::decrease_queue_counter(){
	num_frames_in_reconstruction_queue--;
	this->ui->reco_queued_label->setText(QString::number(num_frames_in_reconstruction_queue));
}

//====================================== BUTTON EVENTS==============================================
void main_window::on_show_rgb_feed_button_clicked() {
	this->rgb_viewer.setVisible(true);
}
void main_window::on_show_depth_feed_button_clicked() {
	this->depth_viewer.setVisible(true);
}
void main_window::on_reco_proc_start_button_clicked(){
	this->reconstruction_worker->run();
}
void main_window::on_reco_proc_pause_button_clicked(){
	this->reconstruction_worker->pause();
}
void main_window::on_reco_play_button_clicked(){
	this->cloud_viewer->run();
}
void main_window::on_reco_pause_button_clicked(){
	this->cloud_viewer->pause();
}
void main_window::on_reco_rewind_button_clicked(){
	this->reco_output_buffer->go_to_frame(0);

}

} //end namespace reco
} //end namespace workbench

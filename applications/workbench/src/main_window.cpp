/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
#include <src/main_window.h>
#include "ui_main_window.h"

//datapipe
#include <reco/datapipe/kinect_v2_info.h>
#include <reco/datapipe/freenect2_pipe.h>
#include <QThread>
#include <QDebug>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

// ARPG includes
#include <calibu/Calibu.h>
#include <HAL/Camera/CameraDevice.h>

//OpenCV Includes
#include <opencv2/core/eigen.hpp>

//utils
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>
#include <reco/utils/color_util.h>

namespace reco {
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080
#define DEFAULT_CALIBRATION_FILE_PATH "media/algomorph/Data/reco/calib/pos_D_2_kinects.xml"
#define DEFAULT_LOG_FILE_PATH "/media/algomorph/Data/reco/cap/pos_D_slow_rotating_human_2_kinects_1240_frames.log"
//#define DEFAULT_LOG_FILE_PATH "/media/algomorph/Data/reco/cap/pos_E_moving_human_4_kinects.log"

main_window::main_window() :
		ui(new Ui_main_window),
				rgb_viewer("RGB Feed", NULL),
				pipe_buffer(new utils::optimistic_assignment_swap_buffer<
								std::shared_ptr<hal::ImageArray>>()),
				pipe(new datapipe::freenect2_pipe(pipe_buffer, datapipe::freenect2_pipe::hal_log,
								DEFAULT_LOG_FILE_PATH)),
				reco_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>()),
				reco_output_buffer(new point_cloud_buffer())
{
	ui->setupUi(this);
	set_up_qvtk_window();
	connect_actions();
	//start pipe with default file
	hook_pipe_signals();
	connect(reco_output_buffer.get(), SIGNAL(size_changed(size_t)), this, SLOT(update_reco_label(size_t)));
}

main_window::~main_window() {
	delete ui;
}

void main_window::set_up_qvtk_window(){
	result_viewer.reset(new pcl::visualization::PCLVisualizer("result view", false));
	result_viewer->setCameraPosition(
			0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,   // viewpoint
			0.0, -1.0, 0.0,  // normal
			0.0);            // viewport


	ui->qvtk_widget->SetRenderWindow(result_viewer->getRenderWindow());
	result_viewer->setupInteractor(ui->qvtk_widget->GetInteractor(),
			ui->qvtk_widget->GetRenderWindow());
	ui->qvtk_widget->update();
}


/**
 * Connect actions of menus with the corresponding slot functions
 */

void main_window::connect_actions() {
	connect(ui->action_open_kinect_devices, SIGNAL(triggered()), this, SLOT(open_kinect_devices()));
	connect(ui->action_open_hal_log, SIGNAL(triggered()), this, SLOT(open_hal_log()));
	connect(ui->action_open_image_folder, SIGNAL(triggered()), this, SLOT(open_image_folder()));
	connect(ui->action_open_calibration_file, SIGNAL(triggered()),this, SLOT(open_calibration_file()));
	connect(ui->action_close_stream, SIGNAL(triggered()), this, SLOT(unhook_pipe_signals()));
}

/**
 * Open kinect feed source from actual devices (if possible)
 */
void main_window::open_kinect_devices() {
	unhook_pipe_signals();
	pipe.reset(new datapipe::freenect2_pipe(pipe_buffer, datapipe::freenect2_pipe::kinect2_device));
	hook_pipe_signals();
}
/**
 * Open kinect feed source from hal log file
 */
void main_window::open_hal_log() {
	QString file_name = QFileDialog::getOpenFileName(this, tr("Open Log File"),
			"/media/algomorph/Data/reco/cap/", tr("HAL Log files (*.log)"));

	if (!file_name.isEmpty()) {
		unhook_pipe_signals();
		//TODO: test if QString --> std::string works on windows like this
		pipe.reset(
				new datapipe::freenect2_pipe(pipe_buffer, datapipe::freenect2_pipe::hal_log,
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
 * Load the calibration from file at the given path
 * @param file_path the given path
 */
void main_window::open_calibration_file() {
	QString qfile_path = QFileDialog::getOpenFileName(this, tr("Open Calibration File"),
			"/media/algomorph/Data/reco/calib/", tr("Calibu calibration files (*.xml)"));

	if (pipe_signals_hooked && !qfile_path.isEmpty()) {
		std::string file_path = qfile_path.toStdString();

		//parse intrinsics
		calibration.reset(new calibration_parameters(file_path));

		//to aviod magic numbers
		const int n_channels_per_kinect = datapipe::kinect_v2_info::channels.size();
		const int num_kinects = pipe->get_num_kinects();

		//check against the pipe's number of channels
		if ((int)calibration->get_num_kinects() != pipe->get_num_kinects()) {
			err(std::invalid_argument)
					<< "The number of kinect feeds in the provided calibration file ("
					<< calibration->get_num_kinects() / n_channels_per_kinect
					<< ") does not correspond to the number of kinect feeds in the provided log file (presumably, "
					<< num_kinects << ")." << enderr;
		}
		calibration_loaded = true;

		reconstruction_worker.reset(new reconstructor(reco_input_buffer,reco_output_buffer,calibration));
		toggle_reco_controls();
	}
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
	connect(ui->play_button, SIGNAL(released()), pipe.get(), SLOT(play()));
	//connect the pipe output to viewer
	connect(pipe.get(), SIGNAL(frame()), this,
			SLOT(display_feeds()));
	rgb_viewer.configure_for_pipe(pipe->get_num_channels());
	depth_viewer.configure_for_pipe(pipe->get_num_channels());

	//colors have to be reinitialized per chance the number of kinects has changed
	pipe_signals_hooked = true; //set flag
}

/**
 * Disconnect pipe signals between output/GUI buttons and pipe slots
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
		//this will stop the reconstruction and clear reconstruction output
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
	pipe_buffer->clear();
	pipe->join_thread();
	std::shared_ptr<hal::ImageArray> dummy;
	pipe_buffer->push_back(dummy);
}

/**
 * Displays the current frame from the buffer on the screen
 */
void main_window::display_feeds() {
	std::shared_ptr<hal::ImageArray> images = this->pipe_buffer->pop_front();
	if(images){
		this->rgb_viewer.on_frame(images);
		this->depth_viewer.on_frame(images);
	}
}

void main_window::update_reco_label(size_t value){
	ui->reco_label->setText(QString::number(value));
}

/**
 * Slot for error reporting (eventually, errors emanating from child threads should print the error
 * to stdout or stderr)
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
	if (pipe_signals_hooked) {
		shut_pipe_down();
	}
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
void main_window::on_reco_proc_stop_button_clicked(){
	this->reconstruction_worker->pause();
}

} //end namespace reco
} //end namespace workbench

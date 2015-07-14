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
#include <reco/datapipe/feed_viewer.h>

//qt
#include <QThread>
#include <QDebug>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

namespace reco {
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080
#define DEFAULT_LOG_FILE_PATH "/media/algomorph/Data/reco/cap/pos_E_moving_human_4_kinects.log"


main_window::main_window() :
		ui(new Ui_main_window),
		rgb_viewer("RGB Feed",NULL),
		buffer(new utils::optimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		pipe(new datapipe::freenect2_pipe(buffer,datapipe::freenect2_pipe::hal_log, DEFAULT_LOG_FILE_PATH))
{
	ui->setupUi(this);

	// Set up the QVTK window
	result_viewer.reset (new pcl::visualization::PCLVisualizer ("result view", false));
	ui->qvtk_widget->SetRenderWindow (result_viewer->getRenderWindow ());
	result_viewer->setupInteractor (ui->qvtk_widget->GetInteractor (), ui->qvtk_widget->GetRenderWindow ());
	ui->qvtk_widget->update();

	connect_actions();
	hook_pipe_signals();
	//ui->qvtk_widget->update();

}

main_window::~main_window() {
	delete ui;

}
/**
 * Connect QAction objects to the methods displaythey should trigger
 */
void main_window::connect_actions() {
	connect(ui->action_open_kinect_devices, SIGNAL(triggered()), this, SLOT(open_kinect_devices()));
	connect(ui->action_open_hal_log, SIGNAL(triggered()), this, SLOT(open_hal_log()));
	connect(ui->action_open_image_folder, SIGNAL(triggered()), this, SLOT(open_image_folder()));
}
/**
 * Open kinect feed source from actual devices (if possible)
 */
void main_window::open_kinect_devices() {

}
/**
 * Open kinect feed source from hal log file
 */
void main_window::open_hal_log() {

}
/**
 * Open kinect feed from image folder
 */
void main_window::open_image_folder() {

}

/**
 * Connect pipe signals to output and GUI buttons to pipe slots
 */
void main_window::hook_pipe_signals() {
	//set up error reporting;
	connect(pipe.get(), SIGNAL(error(QString)), this, SLOT(report_error(QString)));
	//connect the play and pause buttons
	connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(pause()));
	connect(ui->play_button, SIGNAL(released()), pipe.get(), SLOT(play()));
	//connect the pipe output to viewer
	connect(pipe.get(), SIGNAL(frame()), this,
			SLOT(display_feeds()));
	rgb_viewer.hook_to_pipe(pipe,datapipe::feed_viewer::feed_type::RGB);
	depth_viewer.hook_to_pipe(pipe,datapipe::feed_viewer::feed_type::DEPTH);
}


void main_window::display_feeds() {
	std::shared_ptr<hal::ImageArray> images = this->buffer->pop_front();
	this->rgb_viewer.on_frame(images);
	this->depth_viewer.on_frame(images);
}

void main_window::report_error(QString string) {
	qDebug() << string;
}

void main_window::closeEvent(QCloseEvent* event) {
	rgb_viewer.close();
	depth_viewer.close();
	pipe->stop();
	buffer->clear();//let one more item onto the queue
	pipe->join_thread();

}

void main_window::on_show_rgb_feed_button_clicked(){
	this->rgb_viewer.setVisible(true);
}
void main_window::on_show_depth_feed_button_clicked(){
	this->depth_viewer.setVisible(true);
}

} //end namespace reco
} //end namespace workbench

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

//qt
#include <QThread>
#include <QDebug>

//HAL


namespace reco {
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080

main_window::main_window() :
		ui(new Ui_main_window),
				kinect_data_thread(NULL),
				pipe()
{
	ui->setupUi(this);
	connect_actions();
}

main_window::~main_window() {
	delete ui;

}
/**
 * Connect QAction objects to the methods they should trigger
 */
void main_window::connect_actions(){
	connect(ui->action_open_kinect_devices, SIGNAL(triggered()), this, SLOT(open_kinect_devices()));
	connect(ui->action_open_hal_log, SIGNAL(triggered()), this, SLOT(ope));
	connect(ui->action_open_image_folder, SIGNAL(triggered()), this, SLOT(open_image_folder()));
}
/**
 * Instantiate the
 */
void main_window::open_kinect_devices(){

}
void main_window::open_hal_log(){

}
void main_window::open_image_folder(){

}
void main_window::on_launch_viewer_button_released() {
	//TODO:introduce a viewer
}

void main_window::hook_kinect_source_to_thread(){
	if (!kinect_data_thread) {
		kinect_data_thread = new QThread;

		pipe.get()->moveToThread(kinect_data_thread);
		//set up error reporting;
		connect(pipe.get(), SIGNAL(error(QString)), this, SLOT(report_error(QString)));
		pipe->hook_to_thread(kinect_data_thread);

		//TODO: connect result stuff
		kinect_data_thread->start();
	}
}

void main_window::on_play_button_released() {
	if(kinect_data_thread){

	}
}

void main_window::report_error(QString string) {
	qDebug() << string;
}

void main_window::closeEvent(QCloseEvent* event) {
	if (kinect_data_thread && !kinect_data_thread->isFinished()) {
		pipe->request_stop();
	}
}

} //end namespace reco
} //end namespace workbench


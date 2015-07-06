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
#include <reco/workbench/kinect_v2_info.h>

//qt
#include <QThread>
#include <QDebug>

//HAL


namespace reco {
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080
#define DEFAULT_LOG_FILE_PATH "/media/algomorph/Data/reco/cap/pos_D_slow_rotating_human.log"

main_window::main_window() :
		ui(new Ui_main_window),
				pipe(new freenect2_pipe(freenect2_pipe::hal_log, DEFAULT_LOG_FILE_PATH))
{
	ui->setupUi(this);
	connect_actions();
	ui->rgb_video_widget->set_blank(kinect_v2_info::rgb_image_width,kinect_v2_info::rgb_image_height);
	hook_kinect_source_to_thread();

}

main_window::~main_window() {
	delete ui;

}
/**
 * Connect QAction objects to the methods they should trigger
 */
void main_window::connect_actions(){
	connect(ui->action_open_kinect_devices, SIGNAL(triggered()), this, SLOT(open_kinect_devices()));
	connect(ui->action_open_hal_log, SIGNAL(triggered()), this, SLOT(open_hal_log()));
	connect(ui->action_open_image_folder, SIGNAL(triggered()), this, SLOT(open_image_folder()));
}
/**
 * Open kinect feed source from actual devices (if possible)
 */
void main_window::open_kinect_devices(){

}
/**
 * Open kinect feed source from hal log file
 */emit
void main_window::open_hal_log(){

}
/**
 * Open kinect feed from image folder
 */
void main_window::open_image_folder(){

}

void main_window::hook_kinect_source_to_thread(){
	if (!kinect_data_thread) {
		kinect_data_thread = new QThread;
		pipe.get()->moveToThread(kinect_data_thread);
		//set up error reporting;
		connect(pipe.get(), SIGNAL(error(QString)), this, SLOT(report_error(QString)));
		pipe->hook_to_thread(kinect_data_thread);
		//connect the play and pause buttons
		connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(request_pause()));
		connect(ui->play_button, SIGNAL(released()), pipe.get(), SLOT(start()));
		//connect the pipe output to viewer
		connect(pipe.get(), SIGNAL(frame(std::vector<cv::Mat>)), this, SLOT(tmp_display_image(std::vector<cv::Mat>)));
		kinect_data_thread->start();
		//pipe->request_pause();
	}
}

void main_window::tmp_display_image(std::vector<cv::Mat> images){
//	using namespace std;
//	std::cout << "Num images: " << images.size() << std::endl;
//	std::cout << "rows X cols: " << images[0].rows << " X " << images[0].cols << endl;
//	std::cout << "type: " << images[0].type() << endl << endl;
	cv::Mat copy = cv::Mat(images[0].rows,images[0].cols,images[0].type());
	images[0].copyTo(copy);
	ui->rgb_video_widget->set_image_fast(copy);
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


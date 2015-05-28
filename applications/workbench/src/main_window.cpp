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


namespace reco{
namespace workbench {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080


main_window::main_window() :
				ui(new Ui_main_window),
				kinect_data_thread(NULL)
	{
	ui->setupUi(this);
}

main_window::~main_window() {
	delete ui;
	if(kinect_data_thread && !kinect_data_thread->isFinished()){
		pipe->request_stop();
	}
}

void main_window::on_launch_viewer_button_released() {
	//TODO:introduce a viewer
}

void main_window::on_start_camera_button_released(){
	kinect_data_thread = new QThread;

	pipe.get()->moveToThread(kinect_data_thread);
	pipe->hook_to_thread(kinect_data_thread);

	//TODO: connect result stuff

	//set up error reporting;
	connect(pipe.get(), SIGNAL(error(QString)),this,SLOT(report_error(QStrig)));

	kinect_data_thread->start();
}

void main_window::report_error(QString string){
	qDebug() << string;
}

void main_window::closeEvent(QCloseEvent* event){
	//viewer.close();
}


}//end namespace reco
}//end namespace workbench


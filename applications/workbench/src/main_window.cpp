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
				video_pipeline_thread(NULL)
	{
	ui->setupUi(this);
}

main_window::~main_window() {
	delete ui;
	if(video_pipeline_thread && !video_pipeline_thread->isFinished()){
		//videoPipeline->requestStop();
	}
}

void main_window::on_launch_viewer_button_released() {
	//viewer.show();
}

void main_window::on_start_camera_button_released(){
	video_pipeline_thread = new QThread;
	/*videoPipeline->moveToThread(videoPipelineThread);

	ui->videoWidget->connectToVideoPipeline(videoPipeline);
	connect(videoPipeline, SIGNAL(resultImageReady(cv::Mat)), ui->imageOutput, SLOT(setImage(const cv::Mat&)));


	connect(videoPipeline, SIGNAL(error(QString)), this, SLOT(reportError(QString)));
	connect(videoPipelineThread, SIGNAL(started()), videoPipeline, SLOT(run()));
	connect(videoPipeline, SIGNAL(finished()), videoPipelineThread, SLOT(quit()));
	connect(videoPipeline, SIGNAL(finished()), videoPipeline, SLOT(deleteLater()));
	connect(videoPipelineThread, SIGNAL(finished()), videoPipelineThread, SLOT(deleteLater()));*/

	video_pipeline_thread->start();
}

void main_window::report_error(QString string){
	qDebug() << string;
}

void main_window::closeEvent(QCloseEvent* event){
	//viewer.close();
}


}//end namespace reco
}//end namespace workbench


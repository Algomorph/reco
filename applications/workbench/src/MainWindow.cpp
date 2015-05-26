/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
#include "MainWindow.h"
#include "ui_MainWindow.h"



//qt
#include <QThread>
#include <QDebug>


namespace augmentarium{
namespace vstar {

#define CAMERA_PX_WIDTH 1920
#define CAMERA_PX_HEIGHT 1080


MainWindow::MainWindow() :
				ui(new Ui_MainWindow),
				viewer(NULL, ApplicationPaths::GetPath(PathKey::MODELS) + "/cow.osg"),
				videoPipelineThread(NULL),
#ifdef USE_IMAGE_FILES
				videoPipeline(new OpenDtamPipeline<augmentarium::video::ImageFileVideoSource>(
						new augmentarium::video::ImageFileVideoSource(FRAME_DIR)))
#else
videoPipeline(new OpenDtamPipeline<augmentarium::video::ImageFileVideoSource>(
		new augmentarium::video::WebcamVideoSource(CAMERA_PX_WIDTH,CAMERA_PX_HEIGHT)))
#endif
	{
	ui->setupUi(this);
}

MainWindow::~MainWindow() {
	delete ui;
	if(videoPipelineThread && !videoPipelineThread->isFinished()){
		videoPipeline->requestStop();
	}
}

void MainWindow::on_launchViewerButton_released() {
	viewer.show();
}

void MainWindow::on_startCameraButton_released(){
	videoPipelineThread = new QThread;
	videoPipeline->moveToThread(videoPipelineThread);

	ui->videoWidget->connectToVideoPipeline(videoPipeline);
	connect(videoPipeline, SIGNAL(resultImageReady(cv::Mat)), ui->imageOutput, SLOT(setImage(const cv::Mat&)));


	connect(videoPipeline, SIGNAL(error(QString)), this, SLOT(reportError(QString)));
	connect(videoPipelineThread, SIGNAL(started()), videoPipeline, SLOT(run()));
	connect(videoPipeline, SIGNAL(finished()), videoPipelineThread, SLOT(quit()));
	connect(videoPipeline, SIGNAL(finished()), videoPipeline, SLOT(deleteLater()));
	connect(videoPipelineThread, SIGNAL(finished()), videoPipelineThread, SLOT(deleteLater()));

	videoPipelineThread->start();
}

void MainWindow::reportError(QString string){
	qDebug() << string;
}

void MainWindow::closeEvent(QCloseEvent* event){
	viewer.close();
}


}//end namespace vstar
}//end namespace augmentarium


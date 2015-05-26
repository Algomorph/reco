/*
 * main_window.h
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

#ifndef HMD_MAIN_WINDOW_H_
#define HMD_MAIN_WINDOW_H_

#pragma once

//Qt
#include <QMainWindow>
//OVR
//#include <OVR.h>

//video
#include <reco/datapipe/VideoWidget.h>
#include <reco/datapipe/WebcamVideoSource.h>
#include <reco/datapipe/ImageFileVideoSource.h>

//OpenCV
#include <opencv2/core/core.hpp>


class Ui_MainWindow;

namespace reco{
namespace workbench{

class MainWindow: public QMainWindow {
	Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();
protected:
	virtual void closeEvent(QCloseEvent* event);
private:
	Ui_MainWindow* ui;
	//OsgOculusWidget viewer; //TODO: replace with whatever qt->Oculus plugin needed by whatever rendering engine we decide to use
	QThread* videoPipelineThread;

private slots:
	void on_launchViewerButton_released();
	void on_startCameraButton_released();
	void reportError(QString string);

};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

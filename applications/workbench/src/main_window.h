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
#include <reco/datapipe/video_widget.h>
#include <reco/datapipe/webcam_video_source.h>
#include <reco/datapipe/image_file_video_source.h>

//OpenCV
#include <opencv2/core/core.hpp>


class Ui_main_window;

namespace reco{
namespace workbench{

class main_window: public QMainWindow {
	Q_OBJECT
public:
	main_window();
	virtual ~main_window();
protected:
	//keep qt naming convention here (override)
	virtual void closeEvent(QCloseEvent* event);
private:
	Ui_main_window* ui;
	//OsgOculusWidget viewer; //TODO: replace with whatever qt->Oculus plugin needed by whatever rendering engine we decide to use
	QThread* video_pipeline_thread;

private slots:
	void on_launch_viewer_button_released();
	void on_start_camera_button_released();
	void report_error(QString string);

};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */
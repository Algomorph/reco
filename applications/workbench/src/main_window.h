/*
 * main_window.h
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

#ifndef RECO_WORKBENCH_MAIN_WINDOW_H_
#define RECO_WORKBENCH_MAIN_WINDOW_H_

#pragma once

//Qt
#include <QMainWindow>

//datapipe
#include <reco/datapipe/video_widget.h>
#include <reco/datapipe/webcam_video_source.h>
#include <reco/datapipe/image_file_video_source.h>

//local
#include "freenect2_pipe.h"

//OpenCV
#include <opencv2/core/core.hpp>

//std
#include <memory>
#include <vector>


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
	QThread* kinect_data_thread = NULL;
	std::unique_ptr<freenect2_pipe> pipe;

	void connect_actions();
	void hook_kinect_source_to_thread();



private slots:
	void on_play_button_released();
	void report_error(QString string);
	void open_kinect_devices();
	void open_hal_log();
	void open_image_folder();
	void tmp_display_image(std::vector<cv::Mat> images);

};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

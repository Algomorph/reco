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

//utils
#include <reco/utils/optimistic_swap_buffer.h>

//local
#include "freenect2_pipe.h"
#include "freenect2_pipe2.h"

//OpenCV
#include <opencv2/core/core.hpp>

//std::
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
	virtual void closeEvent(QCloseEvent* evenuniquet);
private:
	Ui_main_window* ui;
	QThread* kinect_data_thread = NULL;
	std::shared_ptr<utils::swap_buffer<std::vector<cv::Mat>>> buffer;
	//std::shared_ptr<freenect2_pipe2> pipe;
	std::shared_ptr<freenect2_pipe> pipe;

	void connect_actions();
	void hook_kinect_source_to_thread();
	void hook_kinect_source_to_buttons();



private slots:
	void on_play_button_released();
	void report_error(QString string);
	void open_kinect_devices();
	void open_hal_log();
	void open_image_folder();
	void tmp_display_image(std::vector<cv::Mat> images);
	void tmp_display_image2();
	void tmp_display_image3(std::shared_ptr<std::vector<cv::Mat>> images);
	void tmp_display_rgb(cv::Mat rgb);
	void tmp_display_rgb2(const cv::Mat& rgb);


};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

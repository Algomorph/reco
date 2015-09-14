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
#include <reco/datapipe/freenect2_pipe.h>
#include <reco/datapipe/multi_kinect_rgb_viewer.h>
#include <reco/datapipe/multi_kinect_depth_viewer.h>

//utils
#include <reco/utils/swap_buffer.h>

//OpenCV
#include <opencv2/core/core.hpp>

//pcl
#include <pcl/visualization/pcl_visualizer.h>

//std
#include <memory>
#include <vector>

//local
#include "calibration_parameters.h"
#include "reconstructor.h"


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

	//GUI elements
	datapipe::multi_kinect_rgb_viewer rgb_viewer;
	datapipe::multi_kinect_depth_viewer depth_viewer;
	std::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer;

	//objects for data transfer from sensors/log file
	datapipe::freenect2_pipe::buffer_type pipe_buffer;
	std::shared_ptr<datapipe::freenect2_pipe> pipe;

	//program state variables
	bool pipe_signals_hooked;
	bool calibration_loaded;

	//calibration parameters & reconstruction state
	int num_frames_in_reconstruction_queue;
	std::shared_ptr<calibration_parameters> calibration;
	reconstructor::input_buffer_type reco_input_buffer;
	std::shared_ptr<point_cloud_buffer> reco_output_buffer;
	std::unique_ptr<reconstructor> reconstruction_worker;

	//data handling functions
	void hook_pipe_signals();
	//void assign_cloud_colors();
	void shut_pipe_down();

	//GUI functions
	void connect_actions();
	void toggle_reco_controls();
	void set_up_qvtk_window();//called from constructor
	//load calibration file
	void load_calibration(std::string file_path);


private slots:

	void unhook_pipe_signals();
	void on_frame();
	void update_reco_processed_label(size_t value);
	void decrease_queue_counter();

	//for thread error reporting
	void report_error(QString string);

	//menu actions
	void open_kinect_devices();
	void open_hal_log();
	void open_image_folder();
	void open_calibration_file();

	//buttons
	void on_show_rgb_feed_button_clicked();
	void on_show_depth_feed_button_clicked();
	void on_reco_proc_start_button_clicked();
	void on_reco_proc_pause_button_clicked();
	void on_reco_play_button_clicked();
	void on_reco_pause_button_clicked();
	void on_reco_rewind_button_clicked();



};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

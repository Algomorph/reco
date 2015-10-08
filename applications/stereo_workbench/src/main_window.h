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
#include <reco/datapipe/stereo_pipe.h>
#include <reco/datapipe/typedefs.h>

//misc
#include <reco/misc/calibration_parameters.h>

//local
#include "stereo_processor.h"

class Ui_main_window;

namespace reco{
namespace stereo_workbench{

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

	datapipe::frame_buffer_type video_buffer;
	std::unique_ptr<datapipe::stereo_pipe> pipe;
	datapipe::frame_buffer_type stereo_input_buffer;
	datapipe::frame_buffer_type stereo_output_buffer;
	std::shared_ptr<calibu::Rigd> calibration;
	stereo_processor stereo_proc;

	void hook_pipe();
	void unhook_pipe();
	void connect_actions();

private slots:

	void open_image_pair();
	void open_calibration_file();

	void on_save_current_button_clicked();
	void on_rectify_checkbox_clicked();
	void handle_frame();
};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

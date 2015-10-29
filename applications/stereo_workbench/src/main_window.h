/*
 * main_window.h
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

#pragma once

//Qt
#include <QMainWindow>

//datapipe
#include <reco/datapipe/hal_stereo_pipe.h>
#include <reco/datapipe/typedefs.h>
#include <reco/stereo_workbench/stereo_processor.hpp>

//local
#include <reco/stereo_workbench/stereo_matcher_tuning_panel.hpp>

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
	std::unique_ptr<datapipe::hal_stereo_pipe> pipe;
	datapipe::frame_buffer_type stereo_input_buffer;
	datapipe::frame_buffer_type stereo_output_buffer;
	stereo_processor processor;

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

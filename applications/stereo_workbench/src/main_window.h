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

	datapipe::multichannel_pipe::buffer_type video_buffer;
	std::unique_ptr<datapipe::stereo_pipe> pipe;


	void connect_actions();

private slots:
};

}//end namespace workbench
} //end namespace reco

#endif /* HMD_MAIN_WINDOW_H_ */

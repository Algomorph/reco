/*
 * main_window.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */

//local
#include "../ui_main_window.h"
#include "main_window.h"

//datapipe
#include <reco/datapipe/stereo_pipe.h>

//utils
#include <reco/utils/swap_buffer.h>

namespace reco {
namespace stereo_workbench {

main_window::main_window() :
		ui(new Ui_main_window),
		video_buffer(new utils::optimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		pipe(new datapipe::stereo_pipe(video_buffer,datapipe::stereo_pipe::video_files,{
				"/media/algomorph/Data/reco/cap/yi/YDXJ0005.mp4",
				"/media/algomorph/Data/reco/cap/yi/YDXJ0006.mp4"
		}))
{
	ui->setupUi(this);
}

main_window::~main_window() {
	delete ui;
}


/**
 * Connect actions of menus with the corresponding slot functions
 */
void main_window::connect_actions() {

}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {

}
} //end namespace reco
} //end namespace stereo_workbench

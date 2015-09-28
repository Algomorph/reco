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
#include <reco/utils/debug_util.h>

namespace reco {
namespace stereo_workbench {

main_window::main_window() :
		ui(new Ui_main_window),
		video_buffer(new utils::optimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		pipe(new datapipe::stereo_pipe(video_buffer,datapipe::stereo_pipe::video_files,{
				"/media/algomorph/Data/reco/cap/yi/s05l_edit.mp4",
				"/media/algomorph/Data/reco/cap/yi/s05r_edit.mp4"
		}))
{
	ui->setupUi(this);
	connect_actions();
	hook_pipe();
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
 * Connect the pipe to output and all related buttons
 */
void main_window::hook_pipe(){
	//in case previously hooked
	//unhook_pipe();
	ui->stereo_feed_viewer->configure_for_pipe(pipe->get_num_channels());
	//TODO: revise this to trigger the event in viewer directly
	connect(pipe.get(),SIGNAL(frame()), this, SLOT(handle_frame()));
	connect(ui->capture_button, SIGNAL(released()), pipe.get(), SLOT(run()));
	puts("pipe hooked");
}

/**
 * Disconnect existing pipe from everything and destroy it
 */
void main_window::unhook_pipe(){
	if(pipe){
		pipe.reset();
	}
}

/**
 * Triggered on each frame emergent from the pipe
 */
void main_window::handle_frame(){
	ui->stereo_feed_viewer->on_frame(video_buffer->pop_front());
}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {
	unhook_pipe();

}
} //end namespace reco
} //end namespace stereo_workbench

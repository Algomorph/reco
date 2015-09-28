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
				})),
		//stereo_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>()),
		stereo_input_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		stereo_output_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		stereo_proc(stereo_input_buffer,stereo_output_buffer)
{
	ui->setupUi(this);
	ui->disparity_viewer->configure_for_pipe(1);
	connect_actions();
	hook_pipe();
	stereo_proc.run();
	connect(&stereo_proc,SIGNAL(frame(std::shared_ptr<std::vector<cv::Mat>>)),
			ui->disparity_viewer,SLOT(on_frame(std::shared_ptr<std::vector<cv::Mat>>)));
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
	connect(pipe.get(),SIGNAL(frame()), this, SLOT(handle_frame()));
	connect(ui->capture_button, SIGNAL(released()), pipe.get(), SLOT(run()));
}

/**
 * Disconnect existing pipe from everything and destroy it
 * TODO: 078 is this method needed?
 */
void main_window::unhook_pipe(){
	if(pipe){
		disconnect(pipe.get(),0,0,0);
	}
}

/**
 * Triggered on each frame emergent from the pipe
 */
void main_window::handle_frame(){
	std::shared_ptr<hal::ImageArray> images = video_buffer->pop_front();
	stereo_input_buffer->push_back(images);
	ui->stereo_feed_viewer->on_frame(images);
}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {
	unhook_pipe();
	this->stereo_proc.stop();

}
} //end namespace reco
} //end namespace stereo_workbench

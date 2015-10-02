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
						"/media/algomorph/Data/reco/cap/yi/s10l_edit.mp4",
						"/media/algomorph/Data/reco/cap/yi/s10r_edit.mp4"
				},"/media/algomorph/Data/reco/calib/yi/cameras_s05.xml")),
		//stereo_input_buffer(new utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>()),
		stereo_input_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		stereo_output_buffer(new utils::pessimistic_assignment_swap_buffer<std::shared_ptr<hal::ImageArray>>()),
		calibration(calibu::ReadXmlRig("/media/algomorph/Data/reco/calib/yi/cameras_s05.xml")),
		stereo_proc(stereo_input_buffer,stereo_output_buffer,calibration)
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
 * Connect actions of menus & signals of controls with the corresponding slot functions
 */
void main_window::connect_actions() {

	ui->minimum_disparity_slider->setValue(stereo_proc.stereo_matcher.minDisparity);
	ui->number_of_disparities_slider->setValue(stereo_proc.stereo_matcher.numberOfDisparities);
	ui->window_size_slider->setValue(stereo_proc.stereo_matcher.SADWindowSize);
	ui->p1_slider->setValue(stereo_proc.stereo_matcher.P1);
	ui->p2_slider->setValue(stereo_proc.stereo_matcher.P2);
	ui->pre_filter_cap_slider->setValue(stereo_proc.stereo_matcher.preFilterCap);
	ui->uniqueness_ratio_slider->setValue(stereo_proc.stereo_matcher.uniquenessRatio);
	ui->speckle_window_size_slider->setValue(stereo_proc.stereo_matcher.speckleWindowSize);
	ui->speckle_range_slider->setValue(stereo_proc.stereo_matcher.speckleRange);

	ui->minimum_disparity_spin_box->setValue(stereo_proc.stereo_matcher.minDisparity);
	ui->number_of_disparities_spin_box->setValue(stereo_proc.stereo_matcher.numberOfDisparities);
	ui->window_size_spin_box->setValue(stereo_proc.stereo_matcher.SADWindowSize);
	ui->p1_spin_box->setValue(stereo_proc.stereo_matcher.P1);
	ui->p2_spin_box->setValue(stereo_proc.stereo_matcher.P2);
	ui->pre_filter_cap_spin_box->setValue(stereo_proc.stereo_matcher.preFilterCap);
	ui->uniqueness_ratio_spin_box->setValue(stereo_proc.stereo_matcher.uniquenessRatio);
	ui->speckle_window_size_spin_box->setValue(stereo_proc.stereo_matcher.speckleWindowSize);
	ui->speckle_range_spin_box->setValue(stereo_proc.stereo_matcher.speckleRange);

	connect(ui->minimum_disparity_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_minimum_disparity(int)));
	connect(ui->minimum_disparity_slider, SIGNAL(valueChanged(int)), ui->minimum_disparity_spin_box, SLOT(setValue(int)));
	connect(ui->minimum_disparity_spin_box, SIGNAL(valueChanged(int)), ui->minimum_disparity_slider, SLOT(setValue(int)));
	connect(ui->number_of_disparities_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_num_disparities(int)));
	connect(ui->number_of_disparities_slider, SIGNAL(valueChanged(int)), ui->number_of_disparities_spin_box, SLOT(setValue(int)));
	connect(ui->number_of_disparities_spin_box, SIGNAL(valueChanged(int)), ui->number_of_disparities_slider, SLOT(setValue(int)));
	connect(ui->window_size_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_window_size(int)));
	connect(ui->window_size_slider, SIGNAL(valueChanged(int)), ui->window_size_spin_box, SLOT(setValue(int)));
	connect(ui->window_size_spin_box, SIGNAL(valueChanged(int)), ui->window_size_slider, SLOT(setValue(int)));
	connect(ui->p1_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_p1(int)));
	connect(ui->p1_slider, SIGNAL(valueChanged(int)), ui->p1_spin_box, SLOT(setValue(int)));
	connect(ui->p1_spin_box, SIGNAL(valueChanged(int)), ui->p1_slider, SLOT(setValue(int)));
	connect(ui->p2_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_p2(int)));
	connect(ui->p2_slider, SIGNAL(valueChanged(int)), ui->p2_spin_box, SLOT(setValue(int)));
	connect(ui->p2_spin_box, SIGNAL(valueChanged(int)), ui->p2_slider, SLOT(setValue(int)));
	connect(ui->pre_filter_cap_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_pre_filter_cap(int)));
	connect(ui->pre_filter_cap_slider, SIGNAL(valueChanged(int)), ui->pre_filter_cap_spin_box, SLOT(setValue(int)));
	connect(ui->pre_filter_cap_spin_box, SIGNAL(valueChanged(int)), ui->pre_filter_cap_slider, SLOT(setValue(int)));
	connect(ui->uniqueness_ratio_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_uniqueness_ratio(int)));
	connect(ui->uniqueness_ratio_slider, SIGNAL(valueChanged(int)), ui->uniqueness_ratio_spin_box, SLOT(setValue(int)));
	connect(ui->uniqueness_ratio_spin_box, SIGNAL(valueChanged(int)), ui->uniqueness_ratio_slider, SLOT(setValue(int)));
	connect(ui->speckle_window_size_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_speckle_window_size(int)));
	connect(ui->speckle_window_size_slider, SIGNAL(valueChanged(int)), ui->speckle_window_size_spin_box, SLOT(setValue(int)));
	connect(ui->speckle_window_size_spin_box, SIGNAL(valueChanged(int)), ui->speckle_window_size_slider, SLOT(setValue(int)));
	connect(ui->speckle_range_slider, SIGNAL(valueChanged(int)), &stereo_proc, SLOT(set_speckle_range(int)));
	connect(ui->speckle_range_slider, SIGNAL(valueChanged(int)), ui->speckle_range_spin_box, SLOT(setValue(int)));
	connect(ui->speckle_range_spin_box, SIGNAL(valueChanged(int)), ui->speckle_range_slider, SLOT(setValue(int)));

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
	connect(ui->pause_button, SIGNAL(released()), pipe.get(), SLOT(pause()));
}

/**
 * Disconnect existing pipe from everything and destroy it
 * TODO: 078 is this method needed?
 */
void main_window::unhook_pipe(){
	if(pipe){
		disconnect(ui->capture_button,0,0,0);
		disconnect(ui->pause_button,0,0,0);
	}
}

/**
 * Triggered on each frame emergent from the pipe
 */
void main_window::handle_frame(){
	std::shared_ptr<hal::ImageArray> images = video_buffer->pop_front();
	if(images){
		stereo_input_buffer->push_back(images);
		ui->stereo_feed_viewer->on_frame(images);
	}else{
		std::shared_ptr<hal::ImageArray> dummy;
		video_buffer->clear();
		video_buffer->push_back(dummy);//send another dummy to signal end
	}

}

/**
 * On window close, close the extra windows and shut down data transfer
 * @param event window close event
 */
void main_window::closeEvent(QCloseEvent* event) {
	pipe->stop();
	//halt frame consumption
	std::shared_ptr<hal::ImageArray> dummy;
	video_buffer->clear();
	video_buffer->push_back(dummy);//send dummy to signal end
	this->stereo_proc.stop();

}


} //end namespace reco
} //end namespace stereo_workbench

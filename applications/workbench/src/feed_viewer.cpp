/*
 * feed_viewer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

//local
#include "feed_viewer.h"
#include <reco/workbench/kinect_v2_info.h>


namespace reco {
namespace workbench {

feed_viewer::feed_viewer(QString window_title, QWidget* parent):
		QWidget(parent),
		buffer(),
		video_widgets(){
	//UI initial setup
	this->setWindowTitle(window_title);
	this->setMinimumSize(800,600);
	this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	this->setLayout(this->layout);
	//re-show this whenever source is closed
	this->no_source_connected_label->setText("No source connected.");
	this->layout->addWidget(this->no_source_connected_label);
}

feed_viewer::~feed_viewer(){
	this->unhook_from_pipe();
}

//add video widget for the specified channel
void feed_viewer::add_video_widget(int ix_channel){
	datapipe::video_widget* vid_widget = new datapipe::video_widget();
	layout->addWidget(vid_widget);
	this->video_widgets.emplace_back(ix_channel,vid_widget);
}


void feed_viewer::hook_to_pipe(std::shared_ptr<freenect2_pipe> pipe, feed_type type){
	if(this->pipe){
		//if already hooked to a pipe, unhook
		this->unhook_from_pipe();
	}else{
		this->setVisible(false);
	}
	//get rid of the "no source connected" label
	this->layout->removeWidget(this->no_source_connected_label);

	this->pipe = pipe;
	this->buffer = pipe->get_buffer();

	//figure out how to traverse the pipe's channels
	int num_channels = pipe->get_num_channels();
	int step, offset, total_channels_to_show;
	if(type == feed_type::ALL){
		step = 1;
		offset = 0;
		total_channels_to_show = num_channels;
	}else{
		step = 2;
		offset = type;
		total_channels_to_show = num_channels >>1;
	}
	this->video_widgets.reserve(total_channels_to_show);

	//add a video widget for each channel
	for(int ix_channel = 0; ix_channel < num_channels; ix_channel+=step){
		this->add_video_widget(ix_channel+offset);
	}

}

void feed_viewer::unhook_from_pipe(){
	if(this->pipe){
		this->setVisible(false);
		//remove each video widget from the layout and delete it.
		for(std::tuple<int,datapipe::video_widget*> vid_widget_tuple : this->video_widgets){
			datapipe::video_widget* widget = std::get<1>(vid_widget_tuple);
			this->layout->removeWidget(widget);
			delete widget;
		}
		this->video_widgets.clear();
		this->layout->addWidget(this->no_source_connected_label);
		this->pipe.reset();
	}
}

void feed_viewer::on_frame(std::shared_ptr<hal::ImageArray> images){
	for(std::tuple<int,datapipe::video_widget*> vid_widget_tuple : this->video_widgets){
		std::shared_ptr<hal::Image> img = images->at(std::get<0>(vid_widget_tuple));
		std::get<1>(vid_widget_tuple)->set_image_fast(*img);
	}
}

} /* namespace workbench */
} /* namespace reco */

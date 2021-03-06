/*
 * feed_viewer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

//datapipe
#include <reco/datapipe/kinect_v2_info.h>
#include <reco/datapipe/multichannel_viewer.h>

//utils
#include <reco/utils/debug_util.h>

namespace reco {
namespace datapipe {

multichannel_viewer::multichannel_viewer(QWidget* parent, QString window_title):
		QWidget(parent),
		video_widgets(){
	//UI initial setup
	this->setWindowTitle(window_title);
	this->setMinimumSize(400,300);
	this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	this->setLayout(this->layout);
	//re-show this whenever source is closed
	this->no_source_connected_label->setText("Not configured.");
	this->layout->addWidget(this->no_source_connected_label);
}

multichannel_viewer::~multichannel_viewer(){
	this->clear_gui_configuration();
}

//add video widget for the specified channel
void multichannel_viewer::add_video_widget(int ix_channel){
	datapipe::image_widget* vid_widget = new datapipe::image_widget();
	layout->addWidget(vid_widget);
	this->video_widgets.emplace_back(ix_channel,vid_widget);
}

void multichannel_viewer::set_channel_number(int num_channels){
	if(this->configured_for_pipe){
		//if already hooked to a pipe, unhook
		this->clear_gui_configuration();
	}

	//get rid of the "no source connected" label
	this->no_source_connected_label->setVisible(false);

	//select channels from feed
	std::vector<int> channel_selections = this->select_channels(num_channels);


	//add a video widget for each channel
	for(int channel : channel_selections){
		this->add_video_widget(channel);
	}
	this->configured_for_pipe=true;
}

std::vector<int> multichannel_viewer::select_channels(int total_channels){
	std::vector<int> selection;
	for(int i = 0; i < total_channels; i++){
		selection.push_back(i);
	}
	return selection;
}

void multichannel_viewer::set_blank(int width, int height){
	if(configured_for_pipe){
		for(std::tuple<int,datapipe::image_widget*> vid_widget_tuple : this->video_widgets){
			std::get<1>(vid_widget_tuple)->set_blank(width, height);
		}
	}
}

void multichannel_viewer::clear_gui_configuration(){
	if(this->configured_for_pipe){
		//this->setVisible(false);
		//remove each video widget from the layout and delete it.
		for(std::tuple<int,datapipe::image_widget*> vid_widget_tuple : this->video_widgets){
			datapipe::image_widget* widget = std::get<1>(vid_widget_tuple);
			this->layout->removeWidget(widget);
			delete widget;
		}
		this->video_widgets.clear();
		//this->layout->addWidget(this->no_source_connected_label);
		this->no_source_connected_label->setVisible(true);
		configured_for_pipe = false;
	}
}

void multichannel_viewer::on_frame(std::shared_ptr<hal::ImageArray> images){
	for(std::tuple<int,datapipe::image_widget*> vid_widget_tuple : this->video_widgets){
		int channel_index = std::get<0>(vid_widget_tuple);
		std::shared_ptr<hal::Image> img = images->at(channel_index);
		std::get<1>(vid_widget_tuple)->set_bgr_image_fast(*img);
	}
}
void multichannel_viewer::on_frame(std::shared_ptr<std::vector<cv::Mat>> images){
	for(std::tuple<int,datapipe::image_widget*> vid_widget_tuple : this->video_widgets){
		int channel_index = std::get<0>(vid_widget_tuple);
		std::get<1>(vid_widget_tuple)->set_bgr_image_fast(images->at(channel_index));
	}
}


} /* namespace datapipe */
} /* namespace reco */

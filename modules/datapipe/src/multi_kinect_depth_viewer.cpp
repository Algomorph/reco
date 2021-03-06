/*
 * multi_kinect_depth_viewer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

//datapipe
#include <reco/datapipe/multi_kinect_depth_viewer.h>

//utils
#include <reco/utils/debug_util.h>


namespace reco {
namespace datapipe {

multi_kinect_depth_viewer::multi_kinect_depth_viewer(QWidget* parent, QString window_title) :
				offset_channel_viewer<kinect_v2_info::channels.size(),
						kinect_v2_info::channel_type::DEPTH>(parent, window_title) {
}

multi_kinect_depth_viewer::~multi_kinect_depth_viewer() {
}

void multi_kinect_depth_viewer::on_frame(std::shared_ptr<hal::ImageArray> images){
	for(std::tuple<int,datapipe::image_widget*> vid_widget_tuple : this->video_widgets){
		int channel_index = std::get<0>(vid_widget_tuple);
		std::shared_ptr<hal::Image> img = images->at(channel_index);
		cv::Mat img_mat = static_cast<cv::Mat>(*img) / kinect_v2_info::depth_inv_factor;
		std::get<1>(vid_widget_tuple)->set_float_image_fast(img_mat);
	}
}

} /* namespace datapipe */
} /* namespace reco */

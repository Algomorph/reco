/*
 * kinect2_pipe.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/kinect2_pipe.h>
#include <reco/datapipe/typedefs.h>

namespace reco {
namespace datapipe {
/**
 * @brief Primary constructor
 * Constructs the object with the specified buffer and initializes the retrieval based on the kinect2 data source
 * @param buffer a thread-safe buffer object for storing the output RGB & Depth feeds
 * @param source type of the input source
 * @param path necessary when the source is either an ARPG HAL log (in which case, represents path to the log file) or a file folder (in which case, represents the directory)
 */
kinect2_pipe::kinect2_pipe(frame_buffer_type buffer,kinect2_data_source source,
		const std::string& path) :
		multifeed_pipe(buffer,compile_camera_uri(source,path)),
		source(source),
		path(path)
		 {

	std::string cam_uri = compile_camera_uri(source,path);
	//check that we have appropriate number of channels
	//i.e. for multip feeds, the total number of channels must be evenly divisible
	//by the number of channels per feed
	check_channel_number(cam_uri, num_channels);

	//check that the feed sizes match for RGB & depth, for each kinect
	for (int ix_channel; ix_channel < num_channels; ix_channel++) {
		check_channel_dimensions(cam_uri, ix_channel);
	}
}

kinect2_pipe::~kinect2_pipe() {
}

int kinect2_pipe::get_num_kinects(){
	return num_channels / kinect_v2_info::channels.size();
}

std::string kinect2_pipe::compile_camera_uri(kinect2_data_source source, std::string path){
	std::string uri;
	switch (source) {
	case kinect2_data_source::kinect2_device:
		//note: will open all kinects as "single" camera
		uri = "freenect2:[rgb=1,ir=0,depth=1]//";
		break;
	case kinect2_data_source::hal_log:
		uri = "log://" + path;
		break;
	case kinect2_data_source::image_folder:
		//TODO: 820 implementation pending. not sure if "set_camera("file")" is the right way to read image files
		throw reco::utils::not_implemented();
		break;
	default:
		err(std::invalid_argument) << "Unknown value for data source. Got: " << source
				<< enderr;
		break;
	}
	return uri;
}

} //end namespace datapipe
} //end namespace reco


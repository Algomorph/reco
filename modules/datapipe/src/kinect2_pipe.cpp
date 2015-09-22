/*
 * kinect2_pipe.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/kinect2_pipe.h>

namespace reco {
namespace datapipe {
/**
 * @brief Primary constructor
 * Constructs the object with the specified buffer and initializes the retrieval based on the kinect2 data source
 * @param buffer a thread-safe buffer object for storing the output RGB & Depth feeds
 * @param source type of the input source
 * @param path necessary when the source is either an ARPG HAL log (in which case, represents path to the log file) or a file folder (in which case, represents the directory)
 */
kinect2_pipe::kinect2_pipe(multichannel_pipe::buffer_type buffer,kinect2_data_source source,
		const std::string& path) :
		multifeed_pipe(buffer)

		 {
	switch (source) {
	case kinect2_data_source::kinect2_device:
		//note: will open all kinects as "single" camera
		set_camera("freenect2:[rgb=1,ir=0,depth=1]//");
		break;
	case kinect2_data_source::hal_log:
		set_camera("log://" + path);
		break;
	case kinect2_data_source::image_folder:
		//TODO: 820 implementation pending. not sure if "set_camera("file")" is the right way to read image files
		throw reco::utils::not_implemented();
		break;
	default:
		err(std::invalid_argument) << "Unknown value for kinect2_data_source. Got: " << source
				<< enderr;
		break;
	}

}

kinect2_pipe::~kinect2_pipe() {
}

int kinect2_pipe::get_num_kinects(){
	return num_channels / kinect_v2_info::channels.size();
}

} //end namespace datapipe
} //end namespace reco


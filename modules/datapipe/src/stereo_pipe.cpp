/*
 * stereo_pipe.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/stereo_pipe.h>

//util
#include <reco/utils/cpp_exception_util.h>

namespace reco {
namespace datapipe {

stereo_pipe::stereo_pipe(multichannel_pipe::buffer_type buffer,
						stereo_source source,
						std::vector<std::string> paths):
		multichannel_pipe(buffer,compile_camera_uri(source,paths)),
		source(source),
		paths(paths){

}

stereo_pipe::~stereo_pipe(){}


//TODO: 90 these methods should not be abstract, they should be private static methods in the kinect2_pipe class
void stereo_pipe::check_channel_number(const std::string& cam_uri, size_t num_channels){
}

void stereo_pipe::check_channel_dimensions(const std::string& cam_uri, int ix_channel){
}

std::string stereo_pipe::compile_camera_uri(stereo_source source, std::vector<std::string> paths){
	std::string uri;

	if(paths.size() < 2){
		err(std::invalid_argument) << "Expecting at least two paths. Given number of paths: "
				<< paths.size() << enderr;
	}

	switch(source){
	case video_files:
		{
		std::ostringstream str;
		str << "join://";
		for(std::string path : paths){
			str << "opencv://" << path << "&";
		}
		uri = str.str();
		}
		//last '&' is unneeded
		uri = uri.erase(uri.length()-1,1);

		break;
	default:
		err(std::invalid_argument) << "Unknown value for data source. Got: " << source
						<< enderr;
				break;
		break;
	}
	return uri;
}

} /* namespace datapipe */
} /* namespace reco */

/*
 * stereo_pipe.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/hal_stereo_pipe.h>

//util
#include <reco/utils/cpp_exception_util.h>

namespace reco {
namespace datapipe {

hal_stereo_pipe::hal_stereo_pipe(frame_buffer_type buffer,
						stereo_source source,
						std::vector<std::string> paths,
						std::string calibration_file_path,
						bool use_stereo_rectification):
						hal_pipe(buffer,
								compile_camera_uri(source,paths,
										calibration_file_path,
										use_stereo_rectification)),
						source(source),
						paths(paths){}

hal_stereo_pipe::~hal_stereo_pipe(){}

std::string hal_stereo_pipe::compile_camera_uri(stereo_source source,
		std::vector<std::string> paths,
		std::string calibration_file_path,
		bool use_stereo_rectification){
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
	if(!calibration_file_path.empty()){
		uri = "undistort:[file=" + calibration_file_path +"]//" + uri;
	}
	return uri;
}

} /* namespace datapipe */
} /* namespace reco */

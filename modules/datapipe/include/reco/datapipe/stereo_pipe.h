/*
 * stereo_pipe.h
 *
 *  Created on: Sep 22, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_STEREO_PIPE_H_
#define RECO_DATAPIPE_STEREO_PIPE_H_

#include <reco/datapipe/hal_pipe.h>
#include <reco/datapipe/typedefs.h>

namespace reco {
namespace datapipe {

class stereo_pipe:
		public hal_pipe {
public:
	enum stereo_source{
		video_files = 0
	};
	stereo_pipe(frame_buffer_type buffer,
			stereo_source source = video_files,
			std::vector<std::string> video_file_paths = {},
			std::string calibration_file_path = "",
			bool use_stereo_rectification = true);
	virtual ~stereo_pipe();

private:
	static std::string compile_camera_uri(stereo_source source,
			std::vector<std::string> paths,
			std::string calibration_file_path,
			bool use_stereo_rectification);
	stereo_source source;
	std::vector<std::string> paths;
};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_STEREO_PIPE_H_ */

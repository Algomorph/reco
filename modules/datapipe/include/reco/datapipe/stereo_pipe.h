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

#include <reco/datapipe/multichannel_pipe.h>

namespace reco {
namespace datapipe {

class stereo_pipe:
		public multichannel_pipe {
public:
	enum stereo_source{
		video_files = 0
	};
	stereo_pipe(multichannel_pipe::buffer_type buffer, stereo_source source = video_files, std::vector<std::string> paths = {});
	virtual ~stereo_pipe();

protected:
	virtual void check_channel_number(const std::string& cam_uri, size_t num_channels);
	virtual void check_channel_dimensions(const std::string& cam_uri, int ix_channel);

private:
	static std::string compile_camera_uri(stereo_source source, std::vector<std::string> paths);
	stereo_source source;
	std::vector<std::string> paths;
};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_STEREO_PIPE_H_ */

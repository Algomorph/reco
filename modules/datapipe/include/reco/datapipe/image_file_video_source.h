/*
 * FrameVideoSource.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_FRAMEVIDEOSOURCE_H_
#define RECO_DATAPIPE_FRAMEVIDEOSOURCE_H_

#pragma once

//local
#include "video_source.h"

//standard
#include <set>

namespace reco {
namespace datapipe {

/**
 * Retrieves video frame-by-frame from images in a folder on the hard-disk
 */
class image_file_video_source: public video_source {
public:
	image_file_video_source();
	image_file_video_source(std::string directory, bool looping = true);
	virtual ~image_file_video_source();
	virtual bool set_up();
	virtual void tear_down();
	virtual bool capture_frame();
	virtual unsigned get_width();
	virtual unsigned get_height();
	int get_frame_ix();
private:
	int ix_cur_frame;
	std::string directory;
	std::set<std::string>::iterator file_path_iter;
	bool looping;
	std::set<std::string> orderedFilepaths;
};

} /* namespace video */
} /* namespace augmentarium */

#endif /* RECO_DATAPIPE_FRAMEVIDEOSOURCE_H_ */

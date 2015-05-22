/*
 * FrameVideoSource.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_FRAMEVIDEOSOURCE_H_
#define MODULES_VIDEO_FRAMEVIDEOSOURCE_H_
#pragma once

//local
#include <reco/vidgui/VideoSource.h>

//standard
#include <set>

namespace reco {
namespace vidgui {

/**
 * Retrieves video frame-by-frame from images in a folder on the hard-disk
 */
class ImageFileVideoSource: public VideoSource {
public:
	ImageFileVideoSource();
	ImageFileVideoSource(std::string directory, bool looping = true);
	virtual ~ImageFileVideoSource();
	virtual bool setUp();
	virtual void tearDown();
	virtual cv::Mat retrieveFrame();
	virtual bool trySetResolution(unsigned int width,unsigned int height);
	virtual unsigned getWidth();
	virtual unsigned getHeight();
	int getFrameIx();
private:
	int ixCurFrame;
	std::string directory;
	std::set<std::string>::iterator filePathIter;
	bool looping;
	std::set<std::string> orderedFilepaths;
};

} /* namespace video */
} /* namespace augmentarium */

#endif /* MODULES_VIDEO_FRAMEVIDEOSOURCE_H_ */

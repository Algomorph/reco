/*
 * FrameVideoSource.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <reco/datapipe/ImageFileVideoSource.h>

//std
#include <exception>

namespace reco {
namespace datapipe {

ImageFileVideoSource::ImageFileVideoSource(std::string directory, bool looping) :
				ixCurFrame(-1),
				directory(directory),
				filePathIter(),
				looping(looping)

{

}

ImageFileVideoSource::~ImageFileVideoSource() {
	// TODO Auto-generated destructor stub

}
/**
 * Set's up retrieval of numbered images from a directory.
 * Image file names must end in decimal frame number and extension, and use one of the common image formats, i.e.
 * one of bmp, jpg, tiff, png, pbm, dib.
 * @return always true
 */
bool ImageFileVideoSource::setUp() {
	if (ixCurFrame == -1) {
		using namespace boost::filesystem;
		using namespace std;

		path dir(directory);

		boost::regex filenamePattern("\\D*\\d+[.](bmp|dib|jpg|jpeg|jpe|jp2|png|tiff|tif|pbm|pgm|ppm)",
				boost::regex::extended | boost::regex::icase);

		//traverse directory, insert common image filenames into sorted set
		if (is_directory(dir)) {
			directory_iterator endItr;
			string filename;
			for (directory_iterator dirIter(dir); dirIter != endItr; dirIter++) {
				if (is_regular_file(dirIter->status())
						|| regex_match((filename = dirIter->path().filename().string()), filenamePattern)) {
					orderedFilepaths.insert(dirIter->path().string());
				}
			}
		} else {
			//return false here instead and just make a logging statement?
			throw new std::runtime_error(std::string("Directory \"") + directory + std::string("\" not found."));
		}
	}
	filePathIter = orderedFilepaths.begin();
	ixCurFrame = 0;
	return true;
}

int ImageFileVideoSource::getFrameIx(){
	return ixCurFrame;
}

/**
 *@return width of first image file found in directory assigned at construction
 */
unsigned ImageFileVideoSource::getWidth() {
	if (ixCurFrame == -1) {
		setUp();
	}
	cv::Mat firstIm = cv::imread(*filePathIter);
	unsigned width = firstIm.cols;
	return width;
}
/**
 *@return height of first image file found in directory assigned at construction
 */
unsigned ImageFileVideoSource::getHeight() {
	if (ixCurFrame == -1) {
		setUp();
	}
	std::string curPath = *filePathIter;
	cv::Mat firstIm = cv::imread(curPath);
	unsigned height = firstIm.rows;
	return height;
}

/**
 * Clears the image file path queue.
 */
void ImageFileVideoSource::tearDown() {
	orderedFilepaths.clear();
	ixCurFrame = -1;
}

/**
 * Attempts to retrieve a single image from the queue. Will fail if file has been renamed or deleted.
 * @return matrix with the contents of the retrieved image.
 */
cv::Mat ImageFileVideoSource::retrieveFrame() {
	cv::Mat retval;
	if (filePathIter != orderedFilepaths.end()) {
		retval = cv::imread(*filePathIter);
	} else {
		if (looping) {
			//restart from beginning
			ixCurFrame = 0;
			filePathIter = orderedFilepaths.begin();
			retval = cv::imread(*filePathIter);
		} else {
			//return empty frame, don't advance index or iterator
			return cv::Mat();
		}
	}
	filePathIter++;
	ixCurFrame++;
	return retval;
}

bool ImageFileVideoSource::trySetResolution(unsigned int width, unsigned int hegiht) {
	return false;
}

} /* namespace video */
} /* namespace augmentarium */

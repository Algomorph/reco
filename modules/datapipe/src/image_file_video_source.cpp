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
#include <reco/datapipe/image_file_video_source.h>
#include <exception>

namespace reco {
namespace datapipe {

image_file_video_source::image_file_video_source(std::string directory, bool looping) :
				ix_cur_frame(-1),
				directory(directory),
				file_path_iter(),
				looping(looping)

{

}

image_file_video_source::~image_file_video_source() {}
/**
 * Set's up retrieval of numbered images from a directory.
 * Image file names must end in decimal frame number and extension, and use one of the common image formats, i.e.
 * one of bmp, jpg, tiff, png, pbm, dib.
 * @return always true
 */
bool image_file_video_source::set_up() {
	if (ix_cur_frame == -1) {
		using namespace boost::filesystem;
		using namespace std;

		path dir(directory);

		boost::regex filenamePattern("\\D*\\d+[.](bmp|dib|jpg|jpeg|jpe|jp2|png|tiff|tif|pbm|pgm|ppm)",
				boost::regex::extended | boost::regex::icase);

		//traverse directory, insert common image filenames into sorted set
		if (is_directory(dir)) {
			directory_iterator endItr;
			string filename;
			for (directory_iterator dir_iter(dir); dir_iter != endItr; dir_iter++) {
				if (is_regular_file(dir_iter->status())
						|| regex_match((filename = dir_iter->path().filename().string()), filenamePattern)) {
					orderedFilepaths.insert(dir_iter->path().string());
				}
			}
		} else {
			//return false here instead and just make a logging statement?
			throw new std::runtime_error(std::string("Directory \"") + directory + std::string("\" not found."));
		}
	}
	file_path_iter = orderedFilepaths.begin();
	ix_cur_frame = 0;
	return true;
}

int image_file_video_source::get_frame_ix(){
	return ix_cur_frame;
}

/**
 *@return width of first image file found in directory assigned at construction
 */
unsigned image_file_video_source::get_width() {
	if (ix_cur_frame == -1) {
		set_up();
	}
	cv::Mat first_image = cv::imread(*file_path_iter);
	unsigned width = first_image.cols;
	return width;
}
/**
 *@return height of first image file found in directory assigned at construction
 */
unsigned image_file_video_source::get_height() {
	if (ix_cur_frame == -1) {
		set_up();
	}
	std::string curPafile_path_iterath_iter;
	cv::Mat firstIm = cv::imread(*file_path_iter);
	unsigned height = firstIm.rows;
	return height;
}

/**
 * Clears the image file path queue.
 */
void image_file_video_source::tear_down() {
	orderedFilepaths.clear();
	ix_cur_frame = -1;
}

/**
 * Attempts to retrieve a single image from the queue. Will fail if file has been renamed or deleted.
 * @return matrix with the contents of the retrieved image.
 */
bool image_file_video_source::capture_frame() {
	if (file_path_iter != orderedFilepaths.end()) {
		this->frame = cv::imread(*file_path_iter);
	} else {
		if (looping) {
			//restart from beginning
			ix_cur_frame = 0;
			file_path_iter = orderedFilepaths.begin();
			this->frame = cv::imread(*file_path_iter);
		} else {
			//return empty frame, don't advance index or iterator
			return false;
		}
	}
	file_path_iter++;
	ix_cur_frame++;
	return true;
}

} /* namespace video */
} /* namespace augmentarium */

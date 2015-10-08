/*
 * image_file_pipe.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/datapipe/pipe.h>
#include <opencv2/core/core.hpp>

namespace reco {
namespace datapipe {

class image_file_pipe:
		public pipe {
Q_OBJECT
public:
	image_file_pipe(std::vector<cv::Mat> images, frame_buffer_type buffer);
	virtual ~image_file_pipe();

public slots:
	void push_to_buffer();

private:
	std::vector<cv::Mat> images;



};

} /* namespace datapipe */
} /* namespace reco */

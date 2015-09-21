/*
 * OpenCVVideoFileDriver.h
 *
 *  Created on: Sep 21, 2015
 * Authored by: Gregory Kramida
 */

#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <opencv2/opencv.hpp>

namespace reco {
namespace halext {

using namespace hal;

class OpenCVVideoFileDriver:
		public CameraDriverInterface {
public:
	OpenCVVideoFileDriver(unsigned int nCamId, bool bForceGrey);
	OpenCVVideoFileDriver(const std::string& sFilePath, bool bForceGrey);
	virtual ~OpenCVVideoFileDriver();

	bool Capture(hal::CameraMsg& vImages);
	std::shared_ptr<CameraDriverInterface> GetInputDevice() {
		return std::shared_ptr<CameraDriverInterface>();
	}
	size_t NumChannels() const;
	size_t Width(size_t /*idx*/= 0) const;
	size_t Height(size_t /*idx*/= 0) const;

private:
	void init();
	size_t img_height_;
	size_t img_width_;
	int num_channels_;
	bool force_greyscale_;
	cv::VideoCapture cam_;
};

} /* namespace halext */
} /* namespace reco */

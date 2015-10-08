/*
 * hal_interop.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#include <HAL/Messages/ImageArray.h>
#include <memory>
#include <opencv2/core/core.hpp>

namespace reco{
namespace datapipe{

std::shared_ptr<hal::ImageArray> hal_array_from_cv(std::vector<cv::Mat> matrices);

}//datapipe
}//reco



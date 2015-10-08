/*
 * hal_interop.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/hal_interop.h>
#include <reco/utils/cpp_exception_util.h>

namespace reco {
namespace datapipe {

std::shared_ptr<hal::ImageArray> hal_array_from_cv(std::vector<cv::Mat> matrices) {
	std::shared_ptr<hal::ImageArray> arr = hal::ImageArray::Create();
	for (cv::Mat mat : matrices) {
		if (mat.type() != CV_8UC1 && mat.type() != CV_8UC3) {
			err(std::runtime_error) << "expecting types CV_8UC1 or CV_8UC3. Got: " << mat.type()
					<< enderr;
		}
		hal::ImageMsg* pbImg = arr->Ref().add_image();
		pbImg->set_type(hal::PB_UNSIGNED_BYTE);
		pbImg->set_height(mat.rows);
		pbImg->set_width(mat.cols);
		pbImg->set_format(mat.type() == CV_8UC1 ? hal::PB_LUMINANCE : hal::PB_BGR);

		// This may not store the image in contiguous memory which PbMsgs
		// requires, so we might need to copy it
		cv::Mat cv_image;
		if (!mat.isContinuous()) {
			mat.copyTo(cv_image);
		} else {
			cv_image = mat;
		}
		pbImg->set_data(static_cast<const unsigned char*>(cv_image.data),
				cv_image.elemSize() * cv_image.total());

	}
	return arr;
}

}	//datapipe
}	//reco

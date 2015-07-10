/*
 * camera_driver_interface.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_WORKBENCH_CAMERA_DRIVER_INTERFACE_H_
#define RECO_WORKBENCH_CAMERA_DRIVER_INTERFACE_H_
#pragma once

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Messages/ImageArray.h>

namespace reco{
namespace workbench{


class camera : public hal::Camera {

public:
	   ///////////////////////////////////////////////////////////////
	    bool Capture(
	            std::vector<cv::Mat>& vImages,
	            std::vector<hal::ImageInfoMsg>& vImageInfo
	            ){
	      static std::shared_ptr<hal::ImageArray> pbImages =
	          hal::ImageArray::Create();
	        bool bRes = hal::Camera::Capture( pbImages->Ref() );
	        vImages.resize( pbImages->Size() );
	        vImageInfo.resize( pbImages->Size() );
	        if( bRes ){
	          for (int ii = 0; ii < pbImages->Size(); ++ii) {
	            std::shared_ptr<hal::Image> img = pbImages->at(ii);
	            vImages[ii] = *img;
	            if( img->HasInfo() ){
	              vImageInfo[ii] = img->GetInfo();
	            }
	          }
	        }
	        return bRes;
	    }

};
}//end namespace workbench
}//end namespace reco

#endif /* APPLICATIONS_WORKBENCH_INCLUDE_RECO_WORKBENCH_CAMERA_DRIVER_INTERFACE_H_ */

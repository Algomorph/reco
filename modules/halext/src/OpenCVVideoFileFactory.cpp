/*
 * OpenCVVideoFileFactory.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 */

#include <HAL/Devices/DeviceFactory.h>
#include <reco/halext/OpenCVVideoFileDriver.h>

namespace reco{
namespace halext{

class OpenCVVideoFileFactory : public DeviceFactory<CameraDriverInterface> {
private:
  static const std::string none;

 public:
  OpenCVVideoFileFactory(const std::string& name)
      : DeviceFactory<CameraDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri) {
    bool bGrey            = uri.properties.Get<bool>("grey", false);

    unsigned int nCamId = uri.properties.Get<unsigned int>("id", 0);

    const std::string path = ExpandTildePath(uri.url);
    if(path.empty()){
    	std::string sName     = uri.properties.Get<std::string>("name", "OpenCVCam");
    	return std::shared_ptr<CameraDriverInterface>(
    	        new OpenCVVideoFileDriver(nCamId, bGrey));
    }else{
    	std::string sName     = uri.properties.Get<std::string>("name", "OpenCVVideoFile");
    	return std::shared_ptr<CameraDriverInterface>(
    	    	        new OpenCVVideoFileDriver(path, bGrey));
    }

  }
};

const std::string OpenCVVideoFileFactory::none = "|==NONE==|";

// Register this factory by creating static instance of factory
static OpenCVVideoFileFactory g_OpenCVFactory("opencvvf");

}//end namespace halext
}//end namespace reco





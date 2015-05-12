#ifndef OPENNI_CALIB_SUPPLEMENTARY_HPP
#define OPENNI_CALIB_SUPPLEMENTARY_HPP

#include <opencv2/core/core.hpp>

namespace utl
{
  bool saveOpenNICalibParamsSupplementary ( const std::string filename,
                                            const double depth_ir_dx,
                                            const double depth_ir_dy,
                                            const double depth_scaling
                                          )
  {
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    if (fs.isOpened())
    {
      fs << "depth_ir_dx" << depth_ir_dx;
      fs << "depth_ir_dy" << depth_ir_dy;
      fs << "depth_scaling" << depth_scaling;
      
      fs.release();
      return true;
    }
    
    fs.release();
    return false;
  }

  bool loadOpenNICalibParamsSupplementary ( const std::string filename,
                                            double &depth_ir_dx,
                                            double &depth_ir_dy,
                                            double &depth_scaling
                                          )
  {
    cv::FileStorage fs;
    if (!fs.open(filename, cv::FileStorage::READ))
    {
      fs.release();
      return false;
    }
    
    fs["depth_ir_dx"] >> depth_ir_dx;
    fs["depth_ir_dy"] >> depth_ir_dy;
    fs["depth_scaling"] >> depth_scaling;
    
    fs.release();
    return true;
  }
}


#endif    // OPENNI_DEPTH_CALIB_SUPPLEMENTARY_HPP
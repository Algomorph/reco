#ifndef CV_DEPTH_IMAGE_IO_HPP
#define CV_DEPTH_IMAGE_IO_HPP

#include <fstream>
#include <opencv2/highgui/highgui.hpp>

// Code to read and write opencv float images (CV_32FC1) without loss of precision.
// Adapted from HAL library of ARGG lab (https://github.com/arpg/HAL).

namespace utl
{  
  /** \brief Write an OpenCV image to disk
    * \param[in] filename output filename
    * \param[in] image image
    */ 
  inline
  bool writeDepthImage(const std::string &filename, const cv::Mat &image)
  {
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (file.is_open())
    {
      file << "P7" << std::endl;
      file << image.cols << " " << image.rows << std::endl;
      const size_t size = image.elemSize1() * image.rows * image.cols;
      file << 4294967295 << std::endl;
      file.write((const char*)image.data, size);
      file.close();
      
      return true;
    }
    else
    {
      std::cout << "Could not open file for writing (" << filename << ")" << std::endl;
      return false;
    }
  }
  
  /** \brief Read an image from disk and store it as OpenCV image
    * \param[in] filename output filename
    * \return OpenCV image
    */ 
  inline
  cv::Mat readDepthImage(const std::string &filename)
  {
    // magic number P7, portable depthmap, binary
    std::ifstream file( filename.c_str() );

    unsigned int        nImgWidth;
    unsigned int        nImgHeight;
    long unsigned int   nImgSize;

    cv::Mat imDepth;

    if( file.is_open() ) {
        std::string sType;
        file >> sType;
        file >> nImgWidth;
        file >> nImgHeight;
        file >> nImgSize;

        // the actual PGM/PPM expects this as the next field:
        //    nImgSize++;
        //    nImgSize = (log( nImgSize ) / log(2)) / 8.0;

        // but ours has the actual size (4 bytes of float * pixels):
        nImgSize = 4 * nImgWidth * nImgHeight;

        imDepth = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );

        file.seekg( file.tellg() + (std::ifstream::pos_type)1, std::ios::beg );
        file.read( (char*)imDepth.data, nImgSize );
        file.close();
    }
    return imDepth;
  }
}


#endif    // CV_DEPTH_IMAGE_IO_HPP
#ifndef IMAGE_FILENAME_GENERATOR_HPP
#define IMAGE_FILENAME_GENERATOR_HPP

// STD includes
#include <iostream>

// My includes
#include <cpp_utilities.hpp>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

namespace utl
{

  enum ImageType { RGB, DEPTH, IR, CUSTOM };
  static const char * ImageTypeStrings[] = { "rgb", "depth", "ir", "" };

  ////////////////////////////////////////////////////////////////////////////////
  // Generate a filename from a frame
  std::string generateFilename(int frame_number, const ImageType image_type)
  {
    std::string filename = "frame_" + std::to_string(frame_number) + "_";
    
    switch (image_type)
    {
      case RGB:
        filename += "rgb";
        break;
        
      case DEPTH:
        filename += "depth";
        break;
        
      case IR:
        filename += "ir";
        break;

      case CUSTOM:
        filename += "";
        break;
        
      default:
        std::cout << "Unknown image type" << std::endl;
        filename += "WTF";
    }

    if (image_type != CUSTOM)
      filename += ".png";
    
    return filename;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Generate a filename from a frame
  std::string generateFilename(int frame_number, const std::string &suffix)
  {
    std::string filename = "frame_" + std::to_string(frame_number) + "_" + suffix;
    return filename;
  }
    
  ////////////////////////////////////////////////////////////////////////////////
  // Given a filename figure out if it is a valid frame and if so what is the 
  // frame numer and frame type
  bool getImageInfo(const std::string &filename, int &frame_index, ImageType &image_type)
  {
    // Check if prefix is correct
    if (filename.substr(0, 6) != "frame_")
      return false;
    
    // Get image type
    int startPos = filename.find("_", 6, 1) + 1;
    int endPos   = filename.find(".png");
    std::string imageTypeString = filename.substr(startPos, endPos-startPos);
    
    if (imageTypeString == ImageTypeStrings[RGB])
      image_type = RGB;
    else if (imageTypeString == ImageTypeStrings[DEPTH])
      image_type = DEPTH;
    else if (imageTypeString == ImageTypeStrings[IR])
      image_type = IR;
    else
    {
      std::cout << "[getImageInfo] unknown image type postfix '" << imageTypeString << "'." << std::endl;
      return false;
    }
    
    // Get frame ID
    startPos = 6;
    endPos = filename.find("_", startPos, 1);
    frame_index = std::stoi(filename.substr(startPos, endPos-startPos));
    
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Read frame
  bool readFrame(  const int &frameId,
                          const std::string &inputDir,
                          const ImageType &imageType,
                          cv::Size &imageSize,
                          cv::Mat &imGray
                       )
  {
    // Read image
    std::string filename = generateFilename(frameId, imageType);
    imGray = cv::imread(utl::fullfile(inputDir, filename), CV_LOAD_IMAGE_ANYDEPTH+CV_LOAD_IMAGE_GRAYSCALE);
    
    if (imGray.empty())
      return false;
    
    // If it's an IR image convert to CV_8U
    if (imageType == ImageType::IR)
    {    
      imGray.convertTo(imGray, CV_8U, 1.0);
    }
    
    // Check that image is appropriate size
    if (imageSize == cv::Size())
      imageSize = imGray.size();
    else if (imageSize != imGray.size())
    {
      std::cout << "Error loading image '" << filename << "'. Image size different from previous image sizes" << std::endl;
      std::cout << "Previous image sizes:   " << imageSize << std::endl;
      std::cout << "Current image size: " << imGray.size() << std::endl;
      return false;
    }
    
    return true;
  }
}

#endif  // IMAGE_FILENAME_GENERATOR_HPP
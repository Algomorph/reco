// STD includes
#include <iostream>

// My includes
#include <cpp_utilities.hpp>
#include <cv_depth_tools.hpp>
#include <pcl_cv_conversions.hpp>
#include <kinfu.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

// ARPG includes
#include <calibu/Calibu.h>

//------------------------------------------------------------------------------
// Get command line parameters
//------------------------------------------------------------------------------

bool parseCommandLine(int argc, char** argv, std::string &input_dir, std::string &calibration_file, bool &use_colour)
{
  use_colour = true;
  
  if (pcl::console::parse_argument (argc, argv, "-i", input_dir) < 0)
  {
    std::cout << "You must provide a directory with depth and RGB images." << std::endl;
    return false;
  }

  if (pcl::console::parse_argument (argc, argv, "-c", calibration_file) < 0)
    calibration_file.clear();  

  if (pcl::console::find_switch (argc, argv, "-norgb"))
    use_colour = false;
  
  return true;
}

int main(int argc, char* argv[])
{ 
  //----------------------------------------------------------------------------
  // Get command line parameters
  //----------------------------------------------------------------------------
  
  std::string inputDir;
  std::string calibrationFile;
  bool useColour;
  
  parseCommandLine(argc, argv, inputDir, calibrationFile, useColour);
  
  //----------------------------------------------------------------------------
  // Check that input directory exists
  //----------------------------------------------------------------------------
  
  if (!utl::isDirectory(inputDir))
  {
    std::cout << "Input directory doesn't exist of is not a directory (" << inputDir << ")" << std::endl;
    return -1;
  }

  //----------------------------------------------------------------------------
  // Check image files
  //----------------------------------------------------------------------------

  std::cout << "Reading images from " << inputDir << std::endl;
  
  std::vector<std::string> rgbFilenames;
  std::vector<std::string> depthFilenames;
  
  utl::dir(utl::fullfile(inputDir, "rgb_*.png"), rgbFilenames);
  utl::dir(utl::fullfile(inputDir, "depth_*.pgm"), depthFilenames);
  
  std::cout << rgbFilenames.size() << " RGB files" << std::endl;
  std::cout << depthFilenames.size() << " depth files" << std::endl;
  
  int numFrames = std::min(rgbFilenames.size(), depthFilenames.size());
  
  //----------------------------------------------------------------------------
  // Read first frames to get image properties
  //----------------------------------------------------------------------------

  cv::Mat imDepth   = utl::readDepthImage(utl::fullfile(inputDir, depthFilenames[0]));
  cv::Mat imRGB     = cv::imread(utl::fullfile(inputDir, rgbFilenames[0]));

  int w = imDepth.cols;
  int h = imDepth.rows;
  
  //----------------------------------------------------------------------------
  // Get callibration parameters
  //----------------------------------------------------------------------------
    
  roo::ImageIntrinsics K_depth;

  if (calibrationFile.empty())
  {
    std::cout << "No camera model provided. Using generic camera model based on image dimensions." << std::endl;
        
    const double depth_focal = w * 570.342/640.0;;
    K_depth = roo::ImageIntrinsics(depth_focal,depth_focal, w/2.0 - 0.5, h/2.0 - 0.5 );
  }
  else 
  {
    std::cout << "Using camera model from " << calibrationFile << std::endl;
    std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(calibrationFile);
    Eigen::Matrix3f cam_model = rig->cameras_[0]->K().cast<float>();
    K_depth = roo::ImageIntrinsics(cam_model(0,0), cam_model(1,1), cam_model(0,2), cam_model(1,2));
  }  
 
  //----------------------------------------------------------------------------
  // Initialize kinfu
  //----------------------------------------------------------------------------
  
  int   volRes  = 512;
  float volSize = 1.0;
  float minRayNormalAngle = 75.0;
  float mincostheta = std::cos(minRayNormalAngle * M_PI / 180.0);
  float truncation_factor = 1.0;
  Kinfu kinfu(w, h, useColour, K_depth, volRes, volSize, mincostheta, 0.4, 1.5, truncation_factor);
  
  //----------------------------------------------------------------------------
  // Run kinfu
  //----------------------------------------------------------------------------
   
  int frameIdIt = 0;
  int curFrameId;
  int prevFrameId = -1;
  bool quit = false;
  
  // Loop until we are done
  while (!quit)
  {  
    // Add data to kinfu
    if (!kinfu.getViewonly())
    { 
      cout << "Frame " << frameIdIt << endl;
      
      //------------------------------------------------------------------------
      // Read image files
      //------------------------------------------------------------------------
      
      // Get filenames
      std::string rgbFilename = rgbFilenames[frameIdIt];
      std::string depthFilename = depthFilenames[frameIdIt];
      
      // Get frame numbers
      int rgbFrameId    = std::stoi(rgbFilename.substr(4, 5));
      int depthFrameId  = std::stoi(depthFilename.substr(6, 5));
      
      // Check that RGB and depth frame numbers coincide
      if (rgbFrameId != depthFrameId)
      {
        std::cout << "RGB and depth frames are out of sync." << std::endl;
        std::cout << rgbFilename << std::endl;
        std::cout << depthFilename << std::endl;
        break;
      }
      else
        curFrameId = depthFrameId;
      
      // Check that current frame id is consecutive with previous frame id
      if (prevFrameId == -1)
        prevFrameId = curFrameId;
      else if (curFrameId != prevFrameId+1)
      {
        std::cout << "Frames are not consecutive (" << prevFrameId << " -> " << curFrameId << ")" << std::endl;
        break;
      }
      else
        prevFrameId = curFrameId;
      
      // Read images
      imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilename));
//       imDepth.convertTo(imDepth, CV_16UC1);
      if (useColour)
        imRGB = cv::imread(utl::fullfile(inputDir, rgbFilename));
      
      //------------------------------------------------------------------------
      // Add data to kinfu
      //------------------------------------------------------------------------
      
      kinfu.addFrame(imDepth, imRGB);
            
      // Advance frame
      frameIdIt++;
      
      //------------------------------------------------------------------------
      // Check if we need to quit
      //------------------------------------------------------------------------
      
      if (frameIdIt >= numFrames)
      {
//         kinfu.saveMesh("./mesh.ply");
      }
      
    }
    
    //------------------------------------------------------------------------
    // Display
    //------------------------------------------------------------------------
    
    quit = quit || kinfu.spinOnce();
    cv::imshow("Depth", imDepth / 4500.0);
    if (useColour)
      cv::imshow("RGB", imRGB);
    char k = cv::waitKey(1000);
    
    //------------------------------------------------------------------------
    // Check quit condition
    //------------------------------------------------------------------------
    
    quit = quit || (k == 27);
  }
  
  std::cout << "We are done!" << std::endl;
  
  return 0;
}

// STD includes
#include <iostream>

// My includes
#include <cpp_utilities.hpp>
#include <cv_depth_tools.hpp>
#include <pcl_cv_conversions.hpp>
// #include <pcl_quick_vis.hpp>

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

bool parseCommandLine(int argc, char** argv, std::string &inputDir, std::string &calibrationFile, int& numCams){
	if (pcl::console::parse_argument(argc, argv, "-i", inputDir) < 0){
		std::cout << "You must provide a directory with depth and RGB images." << std::endl;
		return false;
	}

	if (pcl::console::parse_argument(argc, argv, "-c", calibrationFile) < 0){
		calibrationFile.clear();
	}

	if (pcl::console::parse_argument(argc, argv, "-n", numCams) < 0) {
		numCams = 1;
	}

	if (numCams < 1){
		std::cout << "Number of cameras needs to be an integer above zero." << std::endl;
		return false;
	}

	return true;
}

int main(int argc, char* argv[]){
	//----------------------------------------------------------------------------
	// Get command line parameters
	//----------------------------------------------------------------------------

	std::string inputDir;
	std::string calibrationFile;
	int numCams;

	parseCommandLine(argc, argv, inputDir, calibrationFile, numCams);

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
	// Get callibration parameters
	//----------------------------------------------------------------------------

	cv::Mat K_depth(3, 3, CV_32F);

	if (calibrationFile.empty()) {
		std::cout << "No camera model provided. Using generic camera model based on image dimensions." << std::endl;

		cv::Mat imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilenames[0]));
		cv::Mat imRGB = cv::imread(utl::fullfile(inputDir, rgbFilenames[0]));

		const double depth_focal = imDepth.cols * 570.342 / 640.0;
		;
		K_depth.at<float>(0, 0) = depth_focal;
		K_depth.at<float>(1, 1) = depth_focal;
		K_depth.at<float>(0, 2) = imDepth.cols / 2.0 - 0.5;
		K_depth.at<float>(1, 2) = imDepth.rows / 2.0 - 0.5;
	} else {
		std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(calibrationFile);
		Eigen::Matrix3f cam_model = rig->cameras_[0]->K().cast<float>();
		K_depth.at<float>(0, 0) = cam_model(0, 0);
		K_depth.at<float>(1, 1) = cam_model(1, 1);
		K_depth.at<float>(0, 2) = cam_model(0, 2);
		K_depth.at<float>(1, 2) = cam_model(1, 2);
	}

	//----------------------------------------------------------------------------
	// Read and display images
	//----------------------------------------------------------------------------

	// Prepare pcl visualizer
	pcl::visualization::PCLVisualizer visualizer;
//   PclVis visualizer = pcl::createVisualizer();
	visualizer.setCameraPosition(0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,   // viewpoint
			0.0, -1.0, 0.0,   // normal
			0.0);            // viewport

	cv::Mat imRGB;
	cv::Mat imDepth;
	cv::Mat imDepthDiscontinuities;
	cv::Mat imDepthFiltered;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read first RGB & depth images to get sizes
	imRGB = cv::imread(utl::fullfile(inputDir, rgbFilenames[0]));
	imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilenames[0]));
	cv::Size sizeRGB(imRGB.cols,imRGB.rows);
	cv::Size sizeDepth(imDepth.cols,imDepth.rows);

	//Initialize images for display
	cv::Mat RGBCombined = cv::Mat(sizeRGB.height, sizeRGB.width * numCams, CV_8UC3);
	cv::Mat depthCombined = cv::Mat(sizeDepth.height, sizeDepth.width * numCams, imDepth.type());
	cv::Mat depthFilteredCombined = cv::Mat(sizeDepth.height, sizeDepth.width * numCams, imDepth.type());


	for (size_t frameId = 0; frameId < numFrames; frameId++) {

		int RGBColOffset = 0;
		int depthColOffset = 0;

		for (int iKinect = 0; iKinect < numCams; iKinect++){
			// Get filenames
			std::string rgbFilename = rgbFilenames[frameId];
			std::string depthFilename = depthFilenames[frameId];

			// Check actual frame numbers
			int rgbFrameId = std::stoi(rgbFilename.substr(6, 5));
			int depthFrameId = std::stoi(depthFilename.substr(8, 5));

			if (rgbFrameId != depthFrameId) {
				std::cout << "RGB and depth frames are out of sync." << std::endl;
				std::cout << rgbFilename << std::endl;
				std::cout << depthFilename << std::endl;
				break;
			}

			// Read images
			imRGB = cv::imread(utl::fullfile(inputDir, rgbFilename));
			imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilename));

			// Filter depth
			imDepth.copyTo(imDepthFiltered);
			utl::getDepthDiscontinuities(imDepth, imDepthDiscontinuities, 50000.0f);

			for (int x = 0; x < imDepthFiltered.cols; x++) {
				for (int y = 0; y < imDepthFiltered.rows; y++) {
					if (imDepthDiscontinuities.at<uchar>(y, x) == 1)
					imDepthFiltered.at<float>(y, x) = 0.0f;
				}
			}
			//copy over to slices of display images
			cv::Mat sliceRGB(RGBCombined,cv::Rect(RGBColOffset,0,sizeRGB.width, sizeRGB.height));
			imRGB.copyTo(sliceRGB);
			cv::Mat sliceDepth(depthCombined,cv::Rect(depthColOffset,0,sizeDepth.width, sizeDepth.height));
			imDepth.copyTo(sliceDepth);
			cv::Mat sliceDepthFiltered(depthFilteredCombined,cv::Rect(depthColOffset,0,sizeDepth.width, sizeDepth.height));
			imDepthFiltered.copyTo(sliceDepthFiltered);

			//Increase offsets in display images
			RGBColOffset += sizeRGB.width;
			depthColOffset += sizeDepth.width;
		}

		// Display
		cv::imshow("RGB", RGBCombined);
		cv::imshow("Depth", depthCombined / 4500.0f);
		cv::imshow("Depth filtered", depthFilteredCombined / 4500.0f);

		//cv::imshow("Depth discontinuities", imDepthDiscontinuities * 255);
		//cv::imshow("Depth filtered", imDepthFiltered / 4500.0f);

		//TODO: adjust to show point cloud from all depth images combined
		// Convert depth image to cloud
		pcl::cvDepth32F2pclCloud(depthFilteredCombined, K_depth, *cloud);

		if (!visualizer.updatePointCloud(cloud)) {
			visualizer.addPointCloud(cloud);
		}

		visualizer.spinOnce();

		char k = cv::waitKey(1);
		if (k == 27)
		break;
	}

	return 0;
}

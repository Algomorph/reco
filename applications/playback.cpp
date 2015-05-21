// STD includes
#include <iostream>

// Alex's includes
#include <reco/alex/cpp_utilities.hpp>
#include <reco/alex/cv_depth_tools.hpp>
#include <reco/alex/pcl_cv_conversions.hpp>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

// ARPG includes
#include <calibu/Calibu.h>

//define what to display
#define DISPLAY_COMBINED_CLOUD
//#define DISPLAY_MULTI_CLOUD

//------------------------------------------------------------------------------
// Get command line parameters
//------------------------------------------------------------------------------

bool parseCommandLine(int argc, char** argv, std::string &inputDir, std::string &calibrationFile,
		int& numKinects) {
	if (pcl::console::parse_argument(argc, argv, "-i", inputDir) < 0) {
		std::cout << "You must provide a directory with depth and RGB images." << std::endl;
		return false;
	}

	if (pcl::console::parse_argument(argc, argv, "-c", calibrationFile) < 0) {
		calibrationFile.clear();
	}

	if (pcl::console::parse_argument(argc, argv, "-n", numKinects) < 0) {
		numKinects = 1;
	}

	if (numKinects < 1) {
		std::cout << "Number of Kinects needs to be an integer above zero." << std::endl;
		return false;
	}

	return true;
}

template<typename T>
void reorder(std::vector<T>& vec, std::vector<int> order) {
	if (vec.size() != order.size()) {
		std::cout
				<< "[reorder] order vector should have the same size as the vector being reordered."
				<< std::endl;
		exit(EXIT_FAILURE);
	}

	std::vector<T> copy = vec;
	vec.clear();
	for (int ix : order) {
		vec.push_back(copy[ix]);
	}

}

int main(int argc, char* argv[]) {
	//----------------------------------------------------------------------------
	// Get command line parameters
	//----------------------------------------------------------------------------

	std::string inputDir;
	std::string calibrationFile;
	int numKinects;

	parseCommandLine(argc, argv, inputDir, calibrationFile, numKinects);

	//----------------------------------------------------------------------------
	// Check that input directory exists
	//----------------------------------------------------------------------------

	if (!utl::isDirectory(inputDir)) {
		std::cout << "Input directory doesn't exist of is not a directory (" << inputDir << ")"
				<< std::endl;
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

	int numFrames = std::min(rgbFilenames.size(), depthFilenames.size()) / numKinects;

	//----------------------------------------------------------------------------
	// Get callibration parameters
	//----------------------------------------------------------------------------

	std::vector<cv::Mat> depthIntrinsics;
	std::vector<Eigen::Matrix<float, 3, 3>> depthRotations;
	std::vector<Eigen::Matrix<float, 3, 1>> depthTranslations;

	depthIntrinsics.reserve(numKinects);
	depthRotations.reserve(numKinects);
	depthTranslations.reserve(numKinects);

	/*TODO Why do we need to worry about converting the eigen intrinsic & extrinsic matrices to OpenCV format?
	 *  Update linear algebra to use a faster package (See http://nghiaho.com/?p=954 for current comparison.
	 *  Perhaps OpenCV3 will be faster.).*/
	if (calibrationFile.empty()) {
		std::cout
				<< "No camera model provided. Using generic camera models based on image dimensions."
				<< std::endl;
		float curOffset = 0.0F;
		float step = 500.0F;	//mm
		for (int iKinect = 0; iKinect < numKinects; iKinect++) {

			//generate intrinsics
			cv::Mat K_depth(3, 3, CV_32F);
			cv::Mat imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilenames[0]));
			cv::Mat imRGB = cv::imread(utl::fullfile(inputDir, rgbFilenames[0]));

			const double depth_focal = imDepth.cols * 570.342 / 640.0;

			K_depth.at<float>(0, 0) = depth_focal;
			K_depth.at<float>(1, 1) = depth_focal;
			K_depth.at<float>(0, 2) = imDepth.cols / 2.0 - 0.5;
			K_depth.at<float>(1, 2) = imDepth.rows / 2.0 - 0.5;
			depthIntrinsics.push_back(K_depth);

			//generate extrinsics
			cv::Mat P(3, 4, CV_32F);

			//no rotation, identity
			P.at<float>(0, 0) = 1.0F;
			P.at<float>(1, 1) = 1.0F;
			P.at<float>(2, 2) = 1.0F;

			//only x offset
			P.at<float>(3, 0) = curOffset;
			curOffset += step;
		}

	} else {
		//parse intrinsics
		std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(calibrationFile);
		for (int iKinect = 0; iKinect < numKinects; iKinect++) {

			//NOTE: the iKinect*2+1 indexing assumes 2 cameras /Kinect and that depth camera is always after the RGB camera
			Eigen::Matrix3f cam_model = rig->cameras_[iKinect * 2 + 1]->K().cast<float>();
			//convert to opencv
			cv::Mat K_depth(3, 3, CV_32F);
			cv::eigen2cv(cam_model, K_depth);
			depthIntrinsics.push_back(K_depth);
			depthRotations.push_back(
					rig->cameras_[iKinect * 2 + 1]->Pose().rotationMatrix().cast<float>());
			Eigen::Vector3f translation = rig->cameras_[iKinect * 2 + 1]->Pose().translation().cast<
					float>().col(0);
			depthTranslations.push_back(translation);
		}
	}

	//reorder cameras here if needed
	/*std::vector<int> order = {0,1,2};

	 reorder(depthIntrinsics,order);
	 reorder(depthRotations,order);
	 reorder(depthTranslations,order);*/

	//----------------------------------------------------------------------------
	// Read and display images
	//----------------------------------------------------------------------------
	// Prepare pcl visualizer
#ifdef DISPLAY_MULTI_CLOUD
	pcl::visualization::PCLVisualizer visualizer;
	visualizer.setCameraPosition(
			0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,// viewpoint
			0.0, -1.0, 0.0,// normal
			0.0);// viewport
#endif
#ifdef DISPLAY_COMBINED_CLOUD
	pcl::visualization::PCLVisualizer visualizerCombo;
	visualizerCombo.setCameraPosition(0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,   // viewpoint
			0.0, -1.0, 0.0,  // normal
			0.0);            // viewport
#endif

	cv::Mat imRGB;
	cv::Mat imDepth;
	cv::Mat imDepthDiscontinuities;
	cv::Mat imDepthFiltered;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombo(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCombo(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Read first RGB & depth images to get sizes
	imRGB = cv::imread(utl::fullfile(inputDir, rgbFilenames[0]));
	imDepth = utl::readDepthImage(utl::fullfile(inputDir, depthFilenames[0]));
	cv::Size sizeRGB(imRGB.cols, imRGB.rows);
	cv::Size sizeDepth(imDepth.cols, imDepth.rows);

	//Initialize images for display
	cv::Mat RGBCombined = cv::Mat(sizeRGB.height, sizeRGB.width * numKinects, CV_8UC3);
	cv::Mat depthCombined = cv::Mat(sizeDepth.height, sizeDepth.width * numKinects, imDepth.type());
	cv::Mat depthFilteredCombined = cv::Mat(sizeDepth.height, sizeDepth.width * numKinects,
			imDepth.type());

	//add more colors for more kinects
	std::vector<uint32_t> colors = { 0x000000ff, 0x0000ff00, 0x00ff0000 };
	for (size_t frameId = 0; frameId < numFrames; frameId++) {
		cloudCombo->clear();            //clear the cloud
		int RGBColOffset = 0;
		int depthColOffset = 0;

		for (int iKinect = 0; iKinect < numKinects; iKinect++) {
			// Get filenames
			std::string rgbFilename = rgbFilenames[frameId + iKinect * numFrames];
			std::string depthFilename = depthFilenames[frameId + iKinect * numFrames];

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
			cv::Mat sliceRGB(RGBCombined, cv::Rect(RGBColOffset, 0, sizeRGB.width, sizeRGB.height));
			imRGB.copyTo(sliceRGB);
			cv::Mat sliceDepth(depthCombined,
					cv::Rect(depthColOffset, 0, sizeDepth.width, sizeDepth.height));
			imDepth.copyTo(sliceDepth);
			cv::Mat sliceDepthFiltered(depthFilteredCombined,
					cv::Rect(depthColOffset, 0, sizeDepth.width, sizeDepth.height));
			imDepthFiltered.copyTo(sliceDepthFiltered);

			//Increase offsets in display images
			RGBColOffset += sizeRGB.width;
			depthColOffset += sizeDepth.width;
#ifdef DISPLAY_COMBINED_CLOUD
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(new pcl::PointCloud<pcl::PointXYZ>);
			//TODO:Use camera extrinsics **also** to project properly.
			pcl::cvDepth32F2pclCloudColor(imDepth, depthIntrinsics[iKinect],
					depthRotations[iKinect], depthTranslations[iKinect], *cloudCombo,
					colors[iKinect % colors.size()]);
#endif
		}

		// Display
#ifdef DISPLAY_RGB
		cv::imshow("RGB", RGBCombined);
#endif
		cv::imshow("Depth", depthCombined / 4500.0f);
#ifdef DISPLAY_DEPTH_FILTERED
		cv::imshow("Depth filtered", depthFilteredCombined / 4500.0f);
#endif

		//cv::imshow("Depth discontinuities", imDepthDiscontinuities * 255);
		//cv::imshow("Depth filtered", imDepthFiltered / 4500.0f);

		// Convert combined depth image to cloud
#ifdef DISPLAY_MULTI_CLOUD
		pcl::cvDepth32F2pclCloud(depthFilteredCombined, depthIntrinsics[0], *cloud);
		if (!visualizer.updatePointCloud(cloud)) {
			visualizer.addPointCloud(cloud);
		}
		visualizer.spinOnce();
#endif
#ifdef DISPLAY_COMBINED_CLOUD
		if (!visualizerCombo.updatePointCloud(cloudCombo)) {
			visualizerCombo.addPointCloud(cloudCombo);
		}
		visualizerCombo.spinOnce();
#endif

		char k = cv::waitKey(1);
		if (k == 27)
			break;
	}

	return 0;
}

/*
 * extract.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

// Alex's includes
#include <reco/alex/cpp_utilities.hpp>
#include <reco/alex/cv_depth_tools.hpp>

// HAL
#include <HAL/Messages/ImageArray.h>
#include <HAL/Messages/Logger.h>
#include <HAL/Messages/Matrix.h>
#include <HAL/Messages/Reader.h>

//calibu
#include <calibu/Calibu.h>
#include <calibu/cam/camera_models_crtp.h>
#include <pcl/console/parse.h>

//system
#include <iomanip>
#include <cstring>

bool parseCommandLine(int argc, char** argv, std::string& inputFile, std::string& outputDir,
		std::string &calibrationFile) {
	if (pcl::console::parse_argument(argc, argv, "-i", inputFile) < 0) {
		std::cout << "You must provide a path to a HAL log file." << std::endl;
		return false;
	}

	if (pcl::console::parse_argument(argc, argv, "-o", outputDir) < 0) {
		outputDir = inputFile;hal::Reader reader(inputFile);
	}

	if (pcl::console::parse_argument(argc, argv, "-c", calibrationFile) < 0) {
		calibrationFile.clear();
	}

	return true;
}

//------ReadMessage------------------------------------------------------------------------
// Extracts single images out of a log file
//------------------------------------------------------------------------------

/** Extracts posys out of a log file. */
void extractImages(const std::string& log_path, const std::string& outputDir,
		const std::shared_ptr<calibu::Rigd>& rig, const bool doUndistort) {

	std::vector<calibu::LookupTable> lookupTables; //for undistort

	hal::Reader reader(log_path);
	reader.Enable(hal::Msg_Type_Camera);

	int idx = 0;
	std::unique_ptr<hal::Msg> msg;

	//initialize lookup tables for undistort
	if (doUndistort) {
		const size_t num_cams = rig->NumCams();

		lookupTables.resize(num_cams);

		//traverse camera images
		for (int iImage = 0; iImage < rig->cameras_.size(); iImage++) {
			const std::shared_ptr<calibu::CameraInterface<double>> cmod = rig->cameras_[iImage];
			// Setup new camera model
			// For now, assume no change in scale so return same params with no distortion.
			Eigen::Vector2i size_;
			Eigen::VectorXd params_(calibu::LinearCamera<double>::NumParams);
			size_ << cmod->Width(), cmod->Height();
			params_ << cmod->K()(0, 0), cmod->K()(1, 1), cmod->K()(0, 2), cmod->K()(1, 2);
			std::shared_ptr<calibu::CameraInterface<double>> new_cam(
					new calibu::LinearCamera<double>(params_, size_));
			calibu::CreateLookupTable(rig->cameras_[iImage], new_cam->K().inverse(),
					lookupTables[iImage]);
		}
	}
	msg = reader.ReadMessage();

	while (msg) {
		//skip messages w/o camera
		if (msg->has_camera()) {
			const hal::CameraMsg& camMsg = msg->camera();
			int iKinect = 0;
			for (int iCamera = 0; iCamera < camMsg.image_size(); ++iCamera) {
				// Convert frame number to string
				std::string filename;
				std::ostringstream convert;
				convert << std::fixed << std::setfill('0') << std::setw(5) << idx;

				const hal::ImageMsg& imgMsg = camMsg.image(iCamera);

				// Depth image (use custom depth format)
				if (imgMsg.format() == hal::PB_LUMINANCE) {
					filename = utl::fullfile(outputDir,
							"depth_" + std::to_string(iKinect) + "_" + convert.str() + ".pgm");


					cv::Mat imDepth = hal::WriteCvMat(imgMsg);
					if(doUndistort){
						cv::Mat imDepthUndist(imgMsg.height(),imgMsg.width(),CV_32F);
						calibu::Rectify<float>(lookupTables[iCamera], (float*)imDepth.data, (float*)imDepthUndist.data,
								imgMsg.width(), imgMsg.height(), 1);
						imDepth = imDepthUndist;
					}
					utl::writeDepthImage(filename, imDepth);


				} else { // assume RGB image (use standard png image)
					filename = utl::fullfile(outputDir,
							"rgb_" + std::to_string(iKinect) + "_" + convert.str() + ".png");

					if (doUndistort) {
						cv::Mat imRGB(imgMsg.height(), imgMsg.width(), CV_8UC3);
						const int numChannels = 3;
						size_t bufferSize = imgMsg.height() * imgMsg.width() * numChannels;
						uchar buffer[bufferSize];
						memcpy(buffer,imgMsg.data().data(),bufferSize);
						calibu::Rectify<uchar>(lookupTables[iCamera], buffer, imRGB.data,
								imgMsg.width(), imgMsg.height(), numChannels);
						cv::imwrite(filename, imRGB);
					} else {
						//use message data directly
						cv::Mat imRGB(imgMsg.height(), imgMsg.width(), CV_8UC3,
								(void*) imgMsg.data().data());
						cv::imwrite(filename, imRGB);
					}

				}
				iKinect += iCamera % 2;
			}
			++idx;
		}
		msg = reader.ReadMessage();
	}
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
	//----------------------------------------------------------------------------
	// Get command line parameters
	//----------------------------------------------------------------------------

	std::string inputLogPath;
	std::string outputDir;
	std::string calibrationFile;
	bool doUndistort;

	parseCommandLine(argc, argv, inputLogPath, outputDir, calibrationFile);

	//----------------------------------------------------------------------------
	// Check that input file exists
	//----------------------------------------------------------------------------

	if (!utl::isFile(inputLogPath)) {
		std::cout << "Input log file doesn't exist of is not a file (" << inputLogPath << ")"
				<< std::endl;
		return -1;
	}

	//----------------------------------------------------------------------------
	// Load camera intrinsics if available
	//----------------------------------------------------------------------------

	std::shared_ptr<calibu::Rigd> rig;
	if (calibrationFile.empty()) {
		doUndistort = false;
	} else {
		doUndistort = true;
		//parse intrinsics
		rig = calibu::ReadXmlRig(calibrationFile);
	}

	extractImages(inputLogPath, outputDir, rig, doUndistort);
}


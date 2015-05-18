/*
 * extract.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <HAL/Messages/ImageArray.h>
#include <GL/glew.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/rectify_crtp.h>

bool parseCommandLine(int argc, char** argv, std::string& inputFile, std::string& outputDir,
		std::string &calibrationFile, int& numKinects) {
	if (pcl::console::parse_argument(argc, argv, "-i", inputFile) < 0) {
		std::cout << "You must provide a path to a HAL log file." << std::endl;
		return false;
	}

	if (pcl::console::parse_argument(argc, argv, "-o", outputDir) < 0) {
		outputDir = inputFile;
	}

	if (pcl::console::parse_argument(argc, argv, "-c", calibrationFile) < 0) {
		calibrationFile.clear();
	}

	if (pcl::console::parse_argument(argc, argv, "-n", numKinects) < 0) {
		numKinects = 1;
	}

	if (numKinects < 1) {
		std::cout << "Number of kinects needs to be an integer above zero." << std::endl;
		return false;
	}

	return true;
}


hal::ImageMsg* undistort(const calibu::LookupTable& lut){

}

//------------------------------------------------------------------------------
// Extracts single images out of a log file
//------------------------------------------------------------------------------

/** Extracts posys out of a log file. */
void ExtractImages(const std::string& logPath, const std::string& outputDir,
		const std::vector<calibu::LookupTable>& lookupTables,
		const bool doUndistort) {

	hal::Reader reader(logPath);
	reader.Enable(hal::Msg_Type_Camera);

	int idx = 0;
	std::unique_ptr<hal::Msg> msg;

	msg = reader.ReadMessage();
	while (msg) {
		//skip messages w/o camera
		if (msg->has_camera()) {
			const hal::CameraMsg& camMsg = msg->camera();
			int iKinect = 0;
			for (int iImage = 0; iImage < camMsg.image_size(); ++iImage) {
				// Convert frame number to string
				std::string filename;
				std::ostringstream convert;
				convert << std::fixed << std::setfill('0') << std::setw(5) << idx;

				hal::ImageMsg* pimg;
				if (doUndistort) {
					// Retrieve distorted image
					hal::Image inImg = hal::Image(camMsg.image(iImage));

					//create an identical image message
					pimg->set_type((hal::Type) inImg.Type());
					pimg->set_format((hal::Format) inImg.Format());
					uint num_channels = 1;
					if (pimg->format() == hal::PB_LUMINANCE) {
						num_channels = 1;
					} else if (pimg->format() == hal::PB_BGRA || pimg->format() == hal::PB_RGBA) {
						num_channels = 4;
					} else {
						num_channels = 3;
					}

					hal::Image img = hal::Image(*pimg);
					if (pimg->type() == hal::PB_UNSIGNED_BYTE) {
						pimg->mutable_data()->resize(
								inImg.Width() * inImg.Height() * sizeof(unsigned char)
										* num_channels);
						calibu::Rectify<unsigned char>(lookupTables[iImage], inImg.data(),
								reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
								img.Width(), img.Height(), num_channels);
					} else if (pimg->type() == hal::PB_FLOAT) {
						pimg->mutable_data()->resize(
								inImg.Width() * inImg.Height() * sizeof(float) * num_channels);
						calibu::Rectify<float>(lookupTables[iImage], (float*) inImg.data(),
								reinterpret_cast<float*>(&pimg->mutable_data()->front()),
								img.Width(), img.Height(), num_channels);
					}
				} else {
					pimg = &camMsg.image(iImage);
				}

				// Depth image (use custom depth format)
				if (pimg->format() == GL_LUMINANCE) {
					filename = utl::fullfile(outputDir,
							"depth_" + std::to_string(iKinect) + "_" + convert.str() + ".pgm");
					cv::Mat imDepth = hal::WriteCvMat(*pimg);
					utl::writeDepthImage(filename, imDepth);

				} else { // RGB image (use standard png image)
					filename = utl::fullfile(outputDir,
							"rgb_" + std::to_string(iKinect) + "_" + convert.str() + ".png");
					cv::Mat imRGB(pimg->height(), pimg->width(), CV_8UC3,
							(void*) pimg->data().data());
					cv::imwrite(filename, imRGB);
				}
				iKinect += iImage % 2;
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
	int numKinects;

	parseCommandLine(argc, argv, inputLogPath, outputDir, calibrationFile, numKinects);

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

	if (calibrationFile.empty()) {
		doUndistort = false;
	} else {
		doUndistort = true;
		//parse intrinsics
		std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(calibrationFile);
	}

}


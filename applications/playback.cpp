// STD includes
#include <iostream>
#include <stdexcept>

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
#include <HAL/Camera/CameraDevice.h>

// Reco includes
#include <reco/utils/cpp_exception_util.h>

//define what to display
#define DISPLAY_FUSED_CLOUD
//#define DISPLAY_MULTI_CLOUD

#define RGB_CHANNEL_OFFSET 0
#define DEPTH_CHANNEL_OFFSET 1
#define CHANNELS_PER_KINECT 2

/**
 * Set up camera (assumes Kinect v2 source with Depth & IR feeds)
 * @param cam_uri - (in) string uri for HAL camera driver initialization
 * @param camera - (out) camera to be initialized
 * @param num_kinects - (out) # of kinects in the source
 */
void set_camera(const std::string& cam_uri, hal::Camera& camera, int& num_kinects) {
		try {
			camera = hal::Camera(cam_uri);
		} catch (const hal::DeviceException& e) {
			err(std::runtime_error) << "Camera failed to open! Exception: "
					<< e.what() << enderr;
		}

		int num_channels = camera.NumChannels();
		//Assume 2 channels (RGB & depth) per Kinect.
		num_kinects = num_channels / CHANNELS_PER_KINECT;

}

/**
 * Gets the sizes for the rgb & depth feeds
 * @param camera - (in) an initialized Kinect V2 datasource with at least one Kinect RGB & IR feed (2 channels, RGB being first)
 * @param rgb_size - (out) dimensions of the rgb feed
 * @param depth_size - (out) dimensions of the depth feed
 */
void get_image_sizes(const hal::Camera& camera, cv::Size& rgb_size, cv::Size& depth_size){
	rgb_size.width = camera.Width(RGB_CHANNEL_OFFSET);
	rgb_size.height = camera.Height(RGB_CHANNEL_OFFSET);
	depth_size.width = camera.Width(DEPTH_CHANNEL_OFFSET);
	depth_size.height = camera.Height(DEPTH_CHANNEL_OFFSET);
}

//------------------------------------------------------------------------------
// Get command line parameters
//------------------------------------------------------------------------------
bool parse_command_line(int argc, char** argv, std::string &input_file, std::string &calibration_file,
		int& numKinects) {
	if (pcl::console::parse_argument(argc, argv, "-i", input_file) < 0) {
		std::cout << "You must provide a HAL logfile with depth & RGB images." << std::endl;
		return false;
	}

	if (pcl::console::parse_argument(argc, argv, "-c", calibration_file) < 0) {
		calibration_file.clear();
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

	std::string input_file;
	std::string calibration_file;
	int num_kinects;

	parse_command_line(argc, argv, input_file, calibration_file, num_kinects);

	//----------------------------------------------------------------------------
	// Check that input file exists
	//----------------------------------------------------------------------------


	if (!utl::isFile(input_file)) {
		std::cout << "Input file doesn't exist of is not a file (" << input_file << ")"
				<< std::endl;
		return -1;
	}

	//----------------------------------------------------------------------------
	// Read log file
	//----------------------------------------------------------------------------

	std::cout << "Reading kinect v2 feeds from " << input_file << std::endl;

	hal::Camera camera;
	set_camera("log://" + input_file, camera, num_kinects);

	cv::Size2i rgb_size, depth_size;
	get_image_sizes(camera, rgb_size, depth_size);

	//----------------------------------------------------------------------------
	// Get callibration parameters
	//----------------------------------------------------------------------------

	std::vector<cv::Mat> depth_intrinsics;
	std::vector<Eigen::Matrix<float, 3, 3>> depth_rotations;
	std::vector<Eigen::Matrix<float, 3, 1>> depth_translations;

	depth_intrinsics.reserve(num_kinects);
	depth_rotations.reserve(num_kinects);
	depth_translations.reserve(num_kinects);

	/*TODO Why do we need to worry about converting the eigen intrinsic & extrinsic matrices to OpenCV format?
	 *  Update linear algebra to use a faster package (See http://nghiaho.com/?p=954 for current comparison.
	 *  Perhaps OpenCV3 will be faster.).*/
	if (calibration_file.empty()) {
		std::cout
				<< "No camera model provided. Using generic camera models based on image dimensions."
				<< std::endl;
		float curOffset = 0.0F;
		float step = 500.0F;	//mm
		for (int iKinect = 0; iKinect < num_kinects; iKinect++) {

			//generate intrinsics
			cv::Mat K_depth(3, 3, CV_32F);

			const double depth_focal = depth_size.width * 570.342 / 640.0;

			K_depth.at<float>(0, 0) = depth_focal;
			K_depth.at<float>(1, 1) = depth_focal;
			K_depth.at<float>(0, 2) = depth_size.width / 2.0 - 0.5;
			K_depth.at<float>(1, 2) = depth_size.height / 2.0 - 0.5;
			depth_intrinsics.push_back(K_depth);

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
		std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(calibration_file);
		if(rig->cameras_.size()/CHANNELS_PER_KINECT != num_kinects){
			err(std::invalid_argument) << "The number of kinect feeds in the provided calibration file ("
					<< rig->cameras_.size()/CHANNELS_PER_KINECT << ") does not correspond to the number of kinect feeds in the provided log file (presumably, "
					<< num_kinects << ")." << enderr;
		}
		for (int i_kinect = 0; i_kinect < num_kinects; i_kinect++) {

			//NOTE: the iKinect*2+1 indexing assumes 2 cameras /Kinect and that depth camera is always after the RGB camera
			Eigen::Matrix3f cam_model = rig->cameras_[i_kinect * 2 + 1]->K().cast<float>();
			//convert to opencv
			cv::Mat K_depth(3, 3, CV_32F);
			cv::eigen2cv(cam_model, K_depth);
			depth_intrinsics.push_back(K_depth);
			depth_rotations.push_back(
					rig->cameras_[i_kinect * 2 + 1]->Pose().rotationMatrix().cast<float>());
			Eigen::Vector3f translation = rig->cameras_[i_kinect * 2 + 1]->Pose().translation().cast<
					float>().col(0);
			depth_translations.push_back(translation);
		}
	}

	//reorder cameras here if needed
	/*std::vector<int> order = {0,1,2};

	 reorder(depthIntrinsics,order);
	 reorder(depthRotations,order);
	 reorder(depthTranslations,order);*/

	// Prepare pcl visualizer
#ifdef DISPLAY_MULTI_CLOUD
	pcl::visualization::PCLVisualizer visualizer;
	visualizer.setCameraPosition(
			0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,// viewpoint
			0.0, -1.0, 0.0,// normal
			0.0);// viewport
#endif
#ifdef DISPLAY_FUSED_CLOUD
	pcl::visualization::PCLVisualizer visualizer_fused;
	visualizer_fused.setCameraPosition(0.0, 0.0, 0.0,   // camera position
			0.0, 0.0, 1.0,   // viewpoint
			0.0, -1.0, 0.0,  // normal
			0.0);            // viewport
#endif

	cv::Mat im_rgb;
	cv::Mat im_depth;
	cv::Mat im_depth_discontinuities;
	cv::Mat im_depth_filtered;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombo(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fused(new pcl::PointCloud<pcl::PointXYZRGB>);


	//Initialize images for display
	cv::Mat rgb_combined = cv::Mat(rgb_size.height, rgb_size.width * num_kinects, CV_8UC3);
	cv::Mat depth_combined = cv::Mat(depth_size.height, depth_size.width * num_kinects, CV_32F);
	cv::Mat depth_filtered_combined = cv::Mat(depth_size.height, depth_size.width * num_kinects,
			im_depth.type());

	//add more colors for more kinects
	std::vector<uint32_t> colors = { 0x000000ff, 0x0000ff00, 0x00ff0000 };

	std::vector<cv::Mat> images;

	while(camera.Capture(images)){

		cloud_fused->clear();            //clear the fused cloud
		int rgb_col_offset = 0;
		int depth_col_offset = 0;

		for (int i_kinect = 0; i_kinect < num_kinects; i_kinect++) {

			im_rgb = images[RGB_CHANNEL_OFFSET + i_kinect*2];
			im_depth = images[DEPTH_CHANNEL_OFFSET + i_kinect*2];

			// Filter depth
			im_depth.copyTo(im_depth_filtered);
			utl::getDepthDiscontinuities(im_depth, im_depth_discontinuities, 50000.0f);

			for (int x = 0; x < im_depth_filtered.cols; x++) {
				for (int y = 0; y < im_depth_filtered.rows; y++) {
					if (im_depth_discontinuities.at<uchar>(y, x) == 1)
						im_depth_filtered.at<float>(y, x) = 0.0f;
				}
			}
			//copy over to slices of display images
			cv::Mat slice_rgb(rgb_combined, cv::Rect(rgb_col_offset, 0, rgb_size.width, rgb_size.height));
			im_rgb.copyTo(slice_rgb);

			cv::Mat slice_depth(depth_combined,
					cv::Rect(depth_col_offset, 0, depth_size.width, depth_size.height));

			im_depth.copyTo(slice_depth);

			cv::Mat slice_depth_filtered(depth_filtered_combined,
					cv::Rect(depth_col_offset, 0, depth_size.width, depth_size.height));
			im_depth_filtered.copyTo(slice_depth_filtered);

			//Increase offsets in display images
			rgb_col_offset += rgb_size.width;
			depth_col_offset += depth_size.width;
#ifdef DISPLAY_FUSED_CLOUD
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::cvDepth32F2pclCloudColor(im_depth, depth_intrinsics[i_kinect],
					depth_rotations[i_kinect], depth_translations[i_kinect], *cloud_fused,
					colors[i_kinect % colors.size()]);
#endif


		}

		// Display
#ifdef DISPLAY_RGB
		cv::imshow("RGB", rgb_combined);
#endif
		cv::imshow("Depth", depth_combined / 4500.0f);

#ifdef DISPLAY_DEPTH_FILTERED
		cv::imshow("Depth filtered", depth_filtered_combined / 4500.0f);
#endif

		//cv::imshow("Depth discontinuities", imDepthDiscontinuities * 255);
		//cv::imshow("Depth filtered", imDepthFiltered / 4500.0f);

		// Convert combined depth image to cloud
#ifdef DISPLAY_MULTI_CLOUD
		pcl::cvDepth32F2pclCloud(depth_filtered_combined, depth_intrinsics[0], *cloud);
		if (!visualizer.updatePointCloud(cloud)) {
			visualizer.addPointCloud(cloud);
		}
		visualizer.spinOnce();
#endif
#ifdef DISPLAY_FUSED_CLOUD
		if (!visualizer_fused.updatePointCloud(cloud_fused)) {
			visualizer_fused.addPointCloud(cloud_fused);
		}
		visualizer_fused.spinOnce();
#endif

		char k = cv::waitKey(1);
		if (k == 27)
			break;
		images.clear();
	}

	return 0;
}

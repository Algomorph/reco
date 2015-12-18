/*
 * main.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#include <reco/utils/debug_util.h>
#include <reco/utils/cpp_exception_util.h>

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/optflow.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <GL/glew.h>
#include <GL/glut.h>

#include <iostream>
#include <string>
#include <thread>
#include <limits>

#include <pcl/visualization/pcl_visualizer.h>

#include <reco/stereo/opencv_rectifier.hpp>
#include <reco/stereo_workbench/semiglobal_matcher.hpp>
#include <reco/stereo_workbench/pcl_opencv_conversions.hpp>

#define CALIB_FOLDER "/home/algomorph/Dropbox/calib/yi/"

namespace
{
const size_t error_in_command_line = 2;
const size_t success = 0;

} // namespace

namespace reco {
namespace stereo_workbench {
namespace fs = boost::filesystem;
namespace cvxip = cv::ximgproc;

class workbench {

public:
	workbench(fs::path path_left, fs::path path_right, fs::path work_dir, int start_frame = 0,
			int end_frame = std::numeric_limits<int>::max()) :
			start_frame(start_frame), end_frame(end_frame),
					path_l(path_left), path_r(path_right), work_dir(work_dir),
					rectifier(CALIB_FOLDER "0_1_smallboard.xml", 1.0),
					viewer("3D Viewer"){

		if (!fs::is_regular_file(path_l)) {
			err2(std::invalid_argument, "Cannot find file at this path: " << path_l.string());
		}
		if (!fs::is_regular_file(path_r)) {
			err2(std::invalid_argument, "Cannot find file at this path: " << path_r.string());
		}

		if (path_l.string().find(".mp4") != std::string::npos) {
			//videos
			input_is_video = true;

			cap_l.open(path_l.c_str());
			cap_r.open(path_r.c_str());

			//try to obtain the first frame
			if (!cap_l.read(frame_l) || !cap_r.read(frame_r)) {
				err2(std::invalid_argument, "At least one of the videos is empty. Terminating.");
			}
		} else {
			//images
			input_is_video = false;
			frame_l = cv::imread(path_l.string());
			frame_r = cv::imread(path_r.string());
		}
		if (frame_l.type() != frame_r.type() || frame_l.rows != frame_r.rows
				|| frame_l.cols != frame_r.cols) {
			err2(std::invalid_argument,
					"Pixel types or resolutions of input images/videos don't match.");
		}
		set_up_windows(frame_l.size());
		set_up_superpixels(frame_l.size());
		set_up_matcher(frame_l.channels());
	}
	virtual ~workbench() {
		if (input_is_video) {
			cap_l.release();
			cap_r.release();
		}
	}
	void run() {
		cv::Mat result_l, result_r, result_big;



		cv::Mat mask_l;
		if (input_is_video) {
			int i_frame = 0;
			int ch = 0;
			cv::Mat prev, grey, flow;
			cv::cvtColor(frame_l, prev, cv::COLOR_BGR2GRAY);


			cv::Ptr<cv::DenseOpticalFlow> dense_flow = cv::optflow::createOptFlow_DeepFlow();
			//dense_flow->alpha
			while (cap_l.read(frame_l) && cap_r.read(frame_r) && i_frame < end_frame && ch != 27) {
				if (i_frame < start_frame) {
					i_frame++;
					cv::cvtColor(frame_l, prev, cv::COLOR_BGR2GRAY);
					continue;
				}
				cv::cvtColor(frame_l, grey, cv::COLOR_BGR2GRAY);
				cv::calcOpticalFlowFarneback(prev, grey, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
				//dense_flow->calc(prev, grey, flow);
				result_big = flow_to_hsv(flow);


				//cv::optflow::calcOpticalFlowSF(prev,frame_l,flow,4,block_size,max_flow);
//				result_l = process_superpixels(frame_l, seeds_l);
//				result_r = process_superpixels(frame_r, seeds_r);
				cv::imshow(left_win_title, frame_l);
				cv::imshow(right_win_title, frame_l);
				cv::imshow(right_win_title, grey);
				cv::imshow(big_win_title, result_big);
				cv::cvtColor(frame_l, prev, cv::COLOR_BGR2GRAY);

				ch = cv::waitKey(0);
				i_frame++;
			}
		} else {
			rectifier.rectify(frame_l,frame_r,result_l,result_r);




			cv::Mat disparity = cv::imread((work_dir / fs::path("01_LU_disparity.png")).string(), cv::IMREAD_ANYDEPTH);

//			cv::Mat disparity;
//			matcher->compute(result_l, result_r, disparity);
			cv::Mat disparity_mask = cv::imread((work_dir / fs::path("01_LU_disparity_mask.png")).string(), cv::IMREAD_GRAYSCALE);
			cv::Mat mouse_mask = cv::imread((work_dir / fs::path("01_LU_mouse_mask.png")).string(), cv::IMREAD_GRAYSCALE);
			cv::Mat combined_mask;
			cv::bitwise_and(disparity_mask, mouse_mask, combined_mask);

			cv::Mat disparity_32f;
			disparity.convertTo(disparity_32f,CV_32F, 1./16.0);

			cv::Mat Q = rectifier.get_projection_matrix();
			cv::Mat cloud_mat;

			cv::reprojectImageTo3D(disparity_32f, cloud_mat, Q);
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generate_cloud(disparity_uint16, result_l, disparity_mask, mouse_mask, Q);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generate_cloud(disparity, result_l, combined_mask, Q);
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generate_cloud_direct(cloud_mat, result_l, disparity_mask);


			//compute_frame_disparity(result_l, result_r,disparity, result_big);
			//save_disparity_image(work_dir, disparity);


			cv::imshow(left_win_title, result_l);
			cv::imshow(right_win_title, result_r);
			boost::this_thread::sleep (boost::posix_time::microseconds (1000000));

			//cv::Point fixation = cv::Point(frame_l.cols/2,frame_l.rows/2+100);
			//cv::Point fixation = cv::Point(870,670);
			//cv::Point fixation = cv::Point(977,454);
			//log_polar_segment(frame_l, fixation, mask_l,result_big);
			//cv::imshow(big_win_title,disparity);

			if (!viewer.updatePointCloud(cloud)) {
				viewer.addPointCloud(cloud);
			}

			while (!viewer.wasStopped()){
				viewer.spinOnce(100);
				boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			}
			cv::waitKey(0);

		}
	}

private:
	int start_frame, end_frame;
	cv::VideoCapture cap_l, cap_r;
	cv::Mat frame_l, frame_r;
	fs::path path_l, path_r, work_dir;
	cv::Ptr<cvxip::SuperpixelSEEDS> seeds_l, seeds_r;
	cv::Ptr<cv::StereoSGBM> matcher;
	bool input_is_video;
	reco::stereo::opencv_rectifier rectifier;
	pcl::visualization::PCLVisualizer viewer;

	const char* left_win_title = "Left";
	const char* right_win_title = "Right";
	const char* big_win_title = "Big";
	const int superpixel_iterations = 5;


	inline void compute_frame_disparity(cv::Mat& rectified_left, cv::Mat& rectified_right, cv::Mat& disparity, cv::Mat& demo) {

		matcher->compute(rectified_left, rectified_right, disparity);

		cv::normalize(disparity, demo, 0, 255, cv::NORM_MINMAX, CV_8U);

	}

	static void save_disparity_image(const fs::path& dir, const cv::Mat& disparity,
			std::string disparity_file_name = "disparity.png",
			std::string mask_file_name = "disparity_mask.png"){
		cv::Mat mask, out;
		cv::threshold(disparity, mask, 0, 255, CV_8UC1);
		disparity.convertTo(out,CV_16UC1);
		cv::imwrite((dir / fs::path(disparity_file_name)).string(), out);
		cv::imwrite((dir / fs::path(mask_file_name)).string(), mask);

	}

	static cv::Mat flow_to_hsv(cv::Mat& flow) {
			cv::Mat hsv, mag, ang, saturation_channel, out;
			std::vector<cv::Mat> flow_channels;
			saturation_channel = cv::Mat(flow.size(), CV_8UC1, cv::Scalar(255));
			cv::split(flow, flow_channels);
			cv::cartToPolar(flow_channels[0], flow_channels[1], mag, ang, true);
			ang *= 0.708333333;
			cv::normalize(ang, ang, 0, 255, cv::NORM_MINMAX);
			cv::normalize(mag, mag, 0, 255, cv::NORM_MINMAX);
			ang.convertTo(ang, CV_8UC1);
			mag.convertTo(mag, CV_8UC1);
			std::vector<cv::Mat> hsv_channels = { ang, saturation_channel, mag };
			cv::merge(hsv_channels, hsv);
			cv::cvtColor(hsv, out, cv::COLOR_HSV2BGR);
			return out;
		}

	static void log_polar_segment(const cv::Mat& image, cv::Point coord, cv::Mat& mask,
			cv::Mat& log_polar, bool generate_demo = true) {
		using namespace cv;
		using namespace std;
		cv::logPolar(image, log_polar, coord, 256, cv::InterpolationFlags::INTER_LINEAR);

		cv::Mat background_model, foreground_model;

		cv::threshold(log_polar, mask, 1, cv::GC_PR_BGD, cv::THRESH_BINARY);
		cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);

		int sure_width = mask.cols / 8;
		int guess_width = mask.cols / 2;

		cv::rectangle(mask, cv::Rect(sure_width, 0, guess_width - sure_width, mask.rows),
				cv::GC_PR_FGD, cv::FILLED);
		cv::rectangle(mask, cv::Rect(0, 0, guess_width, mask.rows), cv::GC_FGD, cv::FILLED);

		cv::grabCut(log_polar, mask, cv::Rect(),
				background_model,
				foreground_model, 5,
				cv::GrabCutModes::GC_INIT_WITH_MASK);

		cv::rectangle(mask, cv::Rect(0, 0, guess_width, mask.rows), cv::GC_PR_FGD, cv::FILLED);
		cv::threshold(mask, mask, cv::GC_PR_BGD, 1, cv::THRESH_BINARY_INV);

		if (generate_demo) {
			vector<vector<Point> > contours;
			cv::Mat mask_bgr;
			cv::normalize(mask, mask_bgr, 0, 127, cv::NORM_MINMAX);
			cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
			cv::cvtColor(mask_bgr, mask_bgr, cv::COLOR_GRAY2BGR);
			cv::addWeighted(log_polar, 1.0, mask_bgr, -0.5, 1.0, log_polar);
			for (size_t i_contour = 0; i_contour < contours.size(); i_contour++) {
				cv::drawContours(log_polar, contours, i_contour, Scalar(0, 255, 30), 1, cv::LINE_8);
			}
		}
		//cv::normalize(seg,seg,0,255,cv::NORM_MINMAX);
	}

	void set_up_windows(cv::Size image_size) {
#define RESIZE_TO_FIT
		const int screen_width = 1920;
		const int screen_height = 1080;
		const int right_offset = 70;			//approx launcher width
		const int separation = 8;
		const int top_offset = 30;
		const int horiz_limit = screen_width - right_offset;

		float factor = (float) (horiz_limit - separation) / (float) (frame_l.cols + frame_r.cols);

		cv::Size window_l_size(frame_l.cols * factor, frame_l.rows * factor);
		cv::Size window_r_size(frame_r.cols * factor, frame_r.rows * factor);
		cv::Size window_big_size(screen_height - top_offset, screen_width);
		cv::Point window_l_pos(right_offset, top_offset);
		cv::Point window_r_pos(right_offset + window_l_size.width + separation, top_offset);
		cv::Point window_big_pos(screen_width, top_offset);

		cv::namedWindow(left_win_title, cv::WINDOW_FREERATIO);
		cv::namedWindow(right_win_title, cv::WINDOW_FREERATIO);
		cv::resizeWindow(left_win_title, window_l_size.width, window_l_size.height);
		cv::resizeWindow(right_win_title, window_r_size.width, window_r_size.height);
#ifdef RESIZE_TO_FIT
		cv::namedWindow(big_win_title, cv::WINDOW_AUTOSIZE);
#else
		cv::namedWindow(big_win_title, cv::WINDOW_KEEPRATIO);
		cv::resizeWindow(big_win_title, window_big_size.width, window_big_size.height);
#endif

		cv::moveWindow(left_win_title, window_l_pos.x, window_l_pos.y);
		cv::moveWindow(right_win_title, window_r_pos.x, window_r_pos.y);
		cv::moveWindow(big_win_title, window_big_pos.x, window_big_pos.y);


		//viewer.setPosition(right_offset,top_offset*2 + window_l_size.height);
		//viewer.setSize(screen_width - right_offset, screen_height - top_offset*2 - window_l_size.height);
		viewer.setPosition(screen_width, top_offset);
		viewer.setSize(screen_width,screen_height - top_offset);
		//setup PCL viewer
		viewer.setCameraPosition(
				0.0, 0.0, 0.0,   // camera position
				0.0, 0.0, 1.0,   // viewpoint
				0.0, -1.0, 0.0,  // normal
				0);              // which viewport
		//viewer.setCameraClipDistances(0.0, 3.0,0);
		viewer.setBackgroundColor(0.1,0.3,0.35);
	}

	inline cv::Mat process_superpixels(const cv::Mat& frame,
			cv::Ptr<cv::ximgproc::SuperpixelSEEDS>& seeds) {
		cv::Mat hsv, out_bg, out_fg, out;
		const cv::Mat color(frame.size(), CV_8UC3, cv::Scalar(0, 255, 0));
		cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
		seeds->iterate(hsv, superpixel_iterations);
		cv::Mat labels;
		seeds->getLabels(labels);
		cv::Mat mask, mask_inv;
		seeds->getLabelContourMask(mask, 2);
		cv::bitwise_not(mask, mask_inv);
		cv::bitwise_and(frame, frame, out_bg, mask_inv);
		cv::bitwise_and(color, color, out_fg, mask);
		cv::add(out_bg, out_fg, out);
		return out;
	}

	void set_up_superpixels(cv::Size image_size) {
		int num_superpixels = 200;
		int num_levels = 4;
		int num_histogram_bins = 6;
		int prior = 2;
		seeds_l = cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
				num_superpixels, num_levels, prior, num_histogram_bins);
		seeds_r = cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
				num_superpixels, num_levels, prior, num_histogram_bins);
	}

	void set_up_matcher(int channel_number = 3, int penalty_factor = 8, int penalty_over_1_factor =
			4) {
		const int max_disparity = 160 * 4;
		const int block_size = 1;
		const int gradient_cost_influence = 15;
		const int uniqueness_margin = 15;			//%

		matcher = cv::StereoSGBM::create(0, max_disparity, block_size,
						penalty_factor * channel_number * block_size * block_size,
						penalty_factor * penalty_over_1_factor * channel_number * block_size * block_size,
						-1, gradient_cost_influence, uniqueness_margin, 0, 0,
						cv::StereoSGBM::MODE_SGBM);

//		matcher = create_semiglobal_matcher(0, max_disparity, block_size,
//				penalty_factor * channel_number * block_size * block_size,
//				penalty_factor * penalty_over_1_factor * channel_number * block_size * block_size,
//				-1, gradient_cost_influence, uniqueness_margin, 0, 0,
//				cv::StereoSGBM::MODE_SGBM, pixel_cost_type::BIRCHFIELD_TOMASI);
	}

};
}			//stereo_workbench
}			//reco

int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace po = boost::program_options;
	namespace fs = boost::filesystem;

	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> input_files;
	string path;
	int end_frame = std::numeric_limits<int>::max();
	int start_frame = 0;
	regular_options.add_options()
	("help", "Print help messages")
	("path,p", po::value<string>(&path)->required(), "Path to the folder to work in.")
	("input-files,i", po::value<std::vector<std::string>>(&input_files)->required(),
			"Names of input videos or images.")
	("end-frame,e", po::value<int>(&end_frame), "Frame to stop in the videos.")
	("start-frame,s", po::value<int>(&start_frame), "Frame to start from (video).");

	po::positional_options_description positional_options;
	positional_options.add("path", 1);
	positional_options.add("input-files", 2);

	try {
		po::store(po::command_line_parser(argc, argv)
				.options(regular_options).positional(positional_options).run(), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Stereo workbench." << std::endl
					<< regular_options << std::endl;
			return success;
		}

		po::notify(vm); // throws on error, so do after help in case
						// there are any problems

		if (input_files.size() != 2) {
			err2(po::error, "Need exactly two input file paths! Got " << input_files.size() << ".");
		}
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return error_in_command_line;
	}

	fs::path work_dir = fs::path(path);
	fs::path path_l = work_dir / fs::path(input_files[0]);
	fs::path path_r = work_dir / fs::path(input_files[1]);

	reco::stereo_workbench::workbench workbench(path_l, path_r, work_dir, start_frame, end_frame);
	workbench.run();

	return success;
}


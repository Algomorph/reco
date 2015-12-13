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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <GL/glew.h>
#include <GL/glut.h>

#include <iostream>
#include <string>
#include <thread>
#include <limits>

#include <reco/stereo/opencv_rectifier.hpp>
#include <reco/stereo_workbench/semiglobal_matcher.hpp>

#define CALIB_FOLDER "/home/algomorph/Dropbox/calib/yi/"

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;

} // namespace

namespace reco{
namespace stereo_workbench{
namespace fs = boost::filesystem;
namespace cvxip = cv::ximgproc;

class workbench{

public:
	workbench(fs::path path_left, fs::path path_right, int max_frames = std::numeric_limits<int>::max()):
		path_l(path_left),path_r(path_right), rectifier(CALIB_FOLDER "0_1_smallboard_redux.xml"){

		if(!fs::is_regular_file(path_l)){
			err2(std::invalid_argument,"Cannot find file at this path: " << path_l.string());
		}
		if(!fs::is_regular_file(path_r)){
			err2(std::invalid_argument,"Cannot find file at this path: " << path_r.string());
		}

		if(path_l.string().find(".mp4") != std::string::npos){
			//videos
			input_is_video = true;

			cap_l.open(path_l.c_str());
			cap_r.open(path_r.c_str());

			//try to obtain the first frame
			if(!cap_l.read(frame_l) || !cap_r.read(frame_r)){
				err2(std::invalid_argument,"At least one of the videos is empty. Terminating.");
			}
		}else{
			//images
			input_is_video = false;
			frame_l = cv::imread(path_l.string());
			frame_r = cv::imread(path_r.string());
		}
		if(frame_l.type() != frame_r.type() || frame_l.rows != frame_r.rows || frame_l.cols != frame_r.cols){
			err2(std::invalid_argument,"Pixel types or resolutions of input images/videos don't match.");
		}
		set_up_windows(frame_l.size());
		set_up_superpixels(frame_l.size());
		set_up_matcher(frame_l.channels());
	}
	virtual ~workbench(){
		if(input_is_video){
			cap_l.release();
			cap_r.release();
		}
	}
	void run(){
		cv::Mat result_l, result_r, result_big;
		if(input_is_video){
			while(cap_l.read(frame_l) && cap_r.read(frame_r)){
				result_l = process_superpixels(frame_l, seeds_l);
				result_r = process_superpixels(frame_r, seeds_r);
				cv::imshow(left_win_title,result_l);
				cv::imshow(right_win_title,result_r);
				cv::waitKey(1);
			}
		}else{
			process_frame(result_l, result_r, result_big);
			cv::imshow(left_win_title,result_l);
			cv::imshow(right_win_title,result_r);
			cv::imshow(big_win_title,result_big);
			cv::waitKey(0);
		}
	}


private:
	cv::VideoCapture cap_l,cap_r;
	cv::Mat frame_l, frame_r;
	fs::path path_l, path_r;
	cv::Ptr<cvxip::SuperpixelSEEDS> seeds_l, seeds_r;
	cv::Ptr<cv::StereoSGBM> matcher;
	bool input_is_video;
	reco::stereo::opencv_rectifier rectifier;

	const char* left_win_title = "Left";
	const char* right_win_title = "Right";
	const char* big_win_title = "Big";
	const int superpixel_iterations = 5;

	void set_up_superpixels(cv::Size image_size){
		int num_superpixels = 200;
		int num_levels = 4;
		int num_histogram_bins = 6;
		int prior = 2;
		seeds_l = cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
				num_superpixels, num_levels, prior, num_histogram_bins);
		seeds_r = cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
				num_superpixels, num_levels, prior, num_histogram_bins);
	}

	void set_up_matcher(int channel_number = 3, int penalty_factor = 8, int penalty_over_1_factor =4){
		const int max_disparity = 160;
		const int block_size = 1;
		const int gradient_influence = 15;
		const int uniqueness_margin = 15;//%

		matcher = create_semiglobal_matcher(0,max_disparity,block_size,
				penalty_factor*channel_number*block_size*block_size,
				penalty_factor*penalty_over_1_factor*channel_number*block_size*block_size,
				-1, gradient_influence,uniqueness_margin,0,0,
				cv::StereoSGBM::MODE_SGBM, pixel_cost_type::DAISY);
	}


	void set_up_windows(cv::Size image_size){

		const int screen_width = 1920;
		const int screen_height = 1080;
		const int right_offset = 70;//approx launcher width
		const int separation = 8;
		const int top_offset = 30;
		const int horiz_limit = screen_width - right_offset;

		float factor = (float)(horiz_limit-separation) / (float)(frame_l.cols + frame_r.cols);

		cv::Size window_l_size(frame_l.cols*factor, frame_l.rows*factor);
		cv::Size window_r_size(frame_r.cols*factor, frame_r.rows*factor);
		cv::Size window_big_size(screen_height - top_offset,screen_width);
		cv::Point window_l_pos(right_offset,top_offset);
		cv::Point window_r_pos(right_offset+window_l_size.width+separation,top_offset);
		cv::Point window_big_pos(screen_width,top_offset);


		cv::namedWindow(left_win_title, cv::WINDOW_FREERATIO);
		cv::namedWindow(right_win_title, cv::WINDOW_FREERATIO);
		cv::namedWindow(big_win_title, cv::WINDOW_AUTOSIZE);
		//cv::namedWindow(big_win_title, cv::WINDOW_KEEPRATIO);
		cv::resizeWindow(left_win_title, window_l_size.width, window_l_size.height);
		cv::resizeWindow(right_win_title, window_r_size.width, window_r_size.height);
		//cv::resizeWindow(big_win_title, window_big_size.width, window_big_size.height);
//		cv::imshow(left_win_title,frame_l);
//		cv::imshow(right_win_title,frame_r);
		cv::moveWindow(left_win_title,window_l_pos.x,window_l_pos.y);
		cv::moveWindow(right_win_title,window_r_pos.x,window_r_pos.y);
		cv::moveWindow(big_win_title,window_big_pos.x,window_big_pos.y);
	}

	inline cv::Mat process_superpixels(const cv::Mat& frame,
			cv::Ptr<cv::ximgproc::SuperpixelSEEDS>& seeds){
		cv::Mat hsv, out_bg, out_fg, out;
		const cv::Mat color(frame.size(), CV_8UC3, cv::Scalar(0,255,0));
		cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
		seeds->iterate(hsv, superpixel_iterations);
		cv::Mat labels;
		seeds->getLabels(labels);
		cv::Mat mask, mask_inv;
		seeds->getLabelContourMask(mask, 2);
		cv::bitwise_not(mask,mask_inv);
		cv::bitwise_and(frame,frame,out_bg,mask_inv);
		cv::bitwise_and(color,color,out_fg,mask);
		cv::add(out_bg,out_fg,out);
		return out;
	}

	inline void process_frame(cv::Mat& result_l, cv::Mat& result_r, cv::Mat& result_big){

		//rectifier.rectify(frame_l,frame_r,result_l,result_r);
		result_l = frame_l;
		result_r = frame_r;
		cv::Mat temp;
		matcher->compute(result_l,result_r,temp);

		cv::normalize(temp, result_big, 0, 255, cv::NORM_MINMAX, CV_8U);
		//matcher->compute(result_l,result_r,result_big);
	}


};
}//stereo_workbench
}//reco

int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace po = boost::program_options;
	namespace fs = boost::filesystem;

	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> input_paths;
	int max_frames = std::numeric_limits<int>::max();
	regular_options.add_options()
			("help", "Print help messages")
			 ("input-paths", po::value<std::vector<std::string>>(&input_paths), "Paths to input videos or images")
			 ("max-frames", po::value<int>(&max_frames),"Maximum frames in the output frame.");


	po::positional_options_description positional_options;
	positional_options.add("input-paths", 2);


	try{
		po::store(po::command_line_parser(argc, argv)
			.options(regular_options).positional(positional_options).run(),vm); // can throw

		/** --help option
		 */
		if (vm.count("help")){
			std::cout << "Stereo workbench." << std::endl
					<< regular_options << std::endl;
			return SUCCESS;
		}

		po::notify(vm); // throws on error, so do after help in case
						// there are any problems

		if(input_paths.size() != 2){
			err2(po::error,"Need exactly two input file paths! Got " << input_paths.size() << ".");
		}
	}catch (po::error& e){
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}


	fs::path path_l(input_paths[0]);
	fs::path path_r(input_paths[1]);

	reco::stereo_workbench::workbench workbench(path_l, path_r);
	workbench.run();

	return SUCCESS;
}



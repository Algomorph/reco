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

#include <iostream>
#include <string>
#include <thread>
#include <limits>


namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;

} // namespace


inline cv::Mat process_superpixels(const cv::Mat& frame, cv::Ptr<cv::ximgproc::SuperpixelSEEDS>& seeds, int num_iterations){
	cv::Mat hsv, out_bg, out_fg, out;
	const cv::Mat color(frame.size(), CV_8UC3, cv::Scalar(0,255,0));
	cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
	seeds->iterate(hsv, num_iterations);
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

int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace po = boost::program_options;
	namespace fs = boost::filesystem;
	namespace cvxip = cv::ximgproc;
	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> videos;
	int max_frames = std::numeric_limits<int>::max();
	regular_options.add_options()
			("help", "Print help messages")
			 ("input-video-paths", po::value<std::vector<std::string>>(&videos), "Paths to input videos")
			 ("max-frames", po::value<int>(&max_frames),"Maximum frames in the output frame.");


	po::positional_options_description positional_options;
	positional_options.add("input-video-paths", 2);
	//positional_options.add("input-video-paths", 2);


	try{
		po::store(po::command_line_parser(argc, argv)
			.options(regular_options).positional(positional_options).run(),vm); // can throw

		/** --help option
		 */
		if (vm.count("help")){
			std::cout << "Video background subtractor." << std::endl
					<< regular_options << std::endl;
			return SUCCESS;
		}

		po::notify(vm); // throws on error, so do after help in case
						// there are any problems

		if(videos.size() != 2){
			err2(po::error,"Need at least two video paths! Got " << videos.size() << ".");
		}
	}catch (po::error& e){
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	fs::path video_path_l(videos[0]);
	fs::path video_path_r(videos[1]);

	cv::VideoCapture cap_l(video_path_l.c_str());
	cv::VideoCapture cap_r(video_path_r.c_str());



	cv::Mat frame_l, frame_r;
	//try to obtain the first frame
	if(!cap_l.read(frame_l) || !cap_r.read(frame_r)){
		dpt("At least one of the videos is empty. Terminating.")
		return SUCCESS;
	}

	const int screen_width = 1920;
	const int right_offset = 70;//approx launcher width
	const int separation = 8;
	const int top_offset = 30;
	const int horiz_limit = screen_width - right_offset;


	float factor = (float)(horiz_limit-separation) / (float)(frame_l.cols + frame_r.cols);
	cv::Size image_size = frame_l.size();

	cv::Size window_l_size(frame_l.cols*factor, frame_l.rows*factor);
	cv::Size window_r_size(frame_r.cols*factor, frame_r.rows*factor);
	cv::Point window_l_pos(right_offset,top_offset);
	cv::Point window_r_pos(right_offset+window_l_size.width+separation,top_offset);

	const char* left_win_title = "Left";
	const char* right_win_title = "Right";


	cv::namedWindow(left_win_title, cv::WINDOW_FREERATIO);
	cv::namedWindow(right_win_title, cv::WINDOW_FREERATIO);
	cv::resizeWindow(left_win_title, window_l_size.width, window_l_size.height);
	cv::resizeWindow(right_win_title, window_r_size.width, window_r_size.height);
	cv::imshow(left_win_title,frame_l);
	cv::imshow(right_win_title,frame_r);
	cv::moveWindow(left_win_title,window_l_pos.x,window_l_pos.y);
	cv::moveWindow(right_win_title,window_r_pos.x,window_r_pos.y);


	//figure out appropriate window scaling

	int num_superpixels = 200;
	int num_iterations = 5;
	int num_levels = 4;
	int num_histogram_bins = 6;
	int prior = 2;

	cv::Ptr<cvxip::SuperpixelSEEDS> seeds_l =
			cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
			num_superpixels, num_levels, prior, num_histogram_bins);
	cv::Ptr<cvxip::SuperpixelSEEDS> seeds_r =
				cvxip::createSuperpixelSEEDS(image_size.width, image_size.height, 3,
				num_superpixels, num_levels, prior, num_histogram_bins);


	while(cap_l.read(frame_l) && cap_r.read(frame_r)){
		cv::Mat superpixel_demo_l = process_superpixels(frame_l, seeds_l, num_iterations);
		cv::Mat superpixel_demo_r = process_superpixels(frame_r, seeds_r, num_iterations);
		//seeds_l->getLabels()

		cv::imshow(left_win_title,superpixel_demo_l);
		cv::imshow(right_win_title,superpixel_demo_r);



		cv::waitKey(1);
	}
	cap_l.release();
	cap_r.release();



	return SUCCESS;
}



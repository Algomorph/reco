/*
 * optical_flow_contours.cpp
 *
 *  Created on: Dec 8, 2015
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

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <reco/utils/debug_util.h>
#include <iostream>
#include <thread>

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 2;
const size_t SUCCESS = 0;
} // namespace

int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace fs = boost::filesystem;
	namespace po = boost::program_options;

	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> videos;
	int max_frames = std::numeric_limits<int>::max();
	std::string output_filename;
	regular_options.add_options()
			("help,h", "Print help messages")
			 ("input-video-path,i", po::value<std::vector<std::string>>(&videos)->required(), "Path to input video")
			 ("max-frames,m", po::value<int>(&max_frames),"Maximum frames in the output file.")
			 ("output,o", po::value<string>(&output_filename)->default_value("output.mp4"), "Output file name.");


	po::positional_options_description positional_options;
	positional_options.add("input-video-path", 1);


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
	}catch (po::error& e){
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	fs::path video_path(videos[0]);
	fs::path output_path  = video_path.parent_path() / fs::path(output_filename);
	cv::VideoCapture cap(video_path.string());
	cv::Mat frame, prev_grey_frame, grey_frame;
	cv::Mat flow, mag, ang, hsv, saturation_channel, bgr;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));


	if(!cap.read(frame)){
		dpt("Video is empty!");
	}

	int num_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
	int i_frame = 0;
	int report_interval = num_frames / 20;
	//std::vector<cv::Point2f> prev_corners;
	//std::vector<cv::Point2f> corners;
	std::vector<int> status;
	std::vector<double> err;

	cv::VideoWriter writer(output_path.string(), CV_FOURCC('X','2','6','4'), cap.get(cv::CAP_PROP_FPS), frame.size(), true);
	writer.set(cv::VIDEOWRITER_PROP_NSTRIPES, std::thread::hardware_concurrency());
	cv::cvtColor(frame,prev_grey_frame,cv::COLOR_BGR2GRAY);
	//cv::goodFeaturesToTrack(prev_grey_frame,prev_corners,100,0.3,7,7);

	cv::TermCriteria term_criteria(cv::TermCriteria::Type::EPS | cv::TermCriteria::Type::COUNT, 10, 0.03);
	saturation_channel = cv::Mat(frame.size(), CV_8UC1,cv::Scalar(255));

	while(cap.read(frame) && i_frame < max_frames){

		cv::cvtColor(frame,grey_frame,cv::COLOR_BGR2GRAY);
		//cv::calcOpticalFlowPyrLK(prev_grey_frame, grey_frame, prev_corners, corners, status, err, cv::Size(15,15), 2, term_criteria);
		cv::calcOpticalFlowFarneback(prev_grey_frame, grey_frame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		//cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel);
		std::vector<cv::Mat> flow_channels;
		cv::split(flow,flow_channels);
		cv::cartToPolar(flow_channels[0],flow_channels[1], mag, ang, true);
		ang *= 0.708333333;
		//cv::normalize(ang,ang,0, 255, cv::NORM_MINMAX);
		cv::normalize(mag,mag, 0, 255, cv::NORM_MINMAX);
		ang.convertTo(ang,CV_8UC1);
		mag.convertTo(mag,CV_8UC1);
		std::vector<cv::Mat> hsv_channels = {ang,saturation_channel,mag};
		cv::merge(hsv_channels,hsv);


		cv::cvtColor(hsv,bgr,cv::COLOR_HSV2BGR);


		writer << bgr;

		if(i_frame % report_interval == 0){
			dpt("Progress: " << i_frame << "/" << num_frames);
		}
		i_frame++;
	}


	cap.release();
	writer.release();


	return SUCCESS;
}

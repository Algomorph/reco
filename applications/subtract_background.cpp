/*
 * subtract_background.cpp
 *
 *  Created on: Nov 30, 2015
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

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/video.hpp>
#include <opencv2/bgsegm.hpp>
#include <opencv2/cudalegacy.hpp>

#include <reco/utils/debug_util.h>
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
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace


int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace po = boost::program_options;
	namespace fs = boost::filesystem;
	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> videos;
	int max_frames = std::numeric_limits<int>::max();
	regular_options.add_options()
			("help", "Print help messages")
			 ("input-video-path", po::value<std::vector<std::string>>(&videos), "Path to input video")
			 ("max-frames", po::value<int>(&max_frames),"Maximum frames in the output frame.");


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
	fs::path output_path  = video_path.parent_path() / fs::path("output.mp4");


	//cv::Ptr<cv::BackgroundSubtractorMOG2> background_model = cv::createBackgroundSubtractorMOG2();
	//cv::Ptr<cv::bgsegm::BackgroundSubtractorMOG> background_model = cv::bgsegm::createBackgroundSubtractorMOG();
	cv::Ptr<cv::bgsegm::BackgroundSubtractorGMG> background_model = cv::bgsegm::createBackgroundSubtractorGMG();
	//cv::Ptr<cv::BackgroundSubtractorKNN> background_model = cv::createBackgroundSubtractorKNN();
	//cv::Ptr<cv::cuda::BackgroundSubtractorFGD> background_model = cv::cuda::BackgroundSubtractorFGD();

	cv::VideoCapture cap(video_path.string());
	cv::Mat frame;
	cv::Mat mask;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));

	if(!cap.read(frame)){
		dpt("Video is empty!");
	}

	int num_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
	int i_frame = 0;
	int report_interval = num_frames / 20;


	cv::VideoWriter writer(output_path.string(), CV_FOURCC('X','2','6','4'), cap.get(cv::CAP_PROP_FPS), frame.size(), false);
	writer.set(cv::VIDEOWRITER_PROP_NSTRIPES, std::thread::hardware_concurrency());

	do{
		background_model->apply(frame,mask);
		cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel);

		writer << mask;

		if(i_frame % report_interval == 0){
			dpt("Progress: " << i_frame << "/" << num_frames);
		}
		i_frame++;

	}while(cap.read(frame) && i_frame < max_frames);

	cap.release();
	writer.release();


	return SUCCESS;
}


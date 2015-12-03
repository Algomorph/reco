#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <reco/utils/debug_util.h>
#include <reco/stereo/opencv_rectifier.hpp>

#define WORK_FOLDER "/home/algomorph/cap/0_1_calib03/"

int main(int argc, char* argv[]){
	using namespace reco::stereo;
	opencv_rectifier rectifier(WORK_FOLDER "0_1_smallboard_redux.xml");
	cv::Mat im_l = cv::imread(WORK_FOLDER "3_left.png");
	cv::Mat im_r = cv::imread(WORK_FOLDER "3_right.png");
	cv::Mat rect_l, rect_r;
	rectifier.rectify(im_l,im_r,rect_l,rect_r);
	cv::imwrite(WORK_FOLDER "3_uleft.png", rect_l);
	cv::imwrite(WORK_FOLDER "3_uright.png", rect_r);
	return 0;
}

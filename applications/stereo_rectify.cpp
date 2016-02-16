#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <reco/utils/debug_util.h>
#include <reco/calib/opencv_rectifier.hpp>

#define WORK_FOLDER "/media/algomorph/Data/reco/stills/E_stills/"

int main(int argc, char* argv[]){
	using namespace reco::calib;
	opencv_rectifier rectifier("/media/algomorph/Data/reco/calib/E_calib/calib.xml");
	cv::Mat im_l = cv::imread(WORK_FOLDER "0L.png");
	cv::Mat im_r = cv::imread(WORK_FOLDER "0R.png");
	cv::Mat rect_l, rect_r;
	rectifier.rectify(im_l,im_r,rect_l,rect_r);
	cv::imwrite(WORK_FOLDER "UL.png", rect_l);
	cv::imwrite(WORK_FOLDER "UR.png", rect_r);
	return 0;
}

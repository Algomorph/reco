//system
#include <iostream>
#include <stdio.h>
#include <memory>
#include <string>

//freenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/threading.h>
#include <libfreenect2/frame_listener_impl.h>

//opencv
#include <opencv2/core/core.hpp>

int main(int argc, char* argv[]){
	libfreenect2::Freenect2 freenect2;

	const int N = freenect2.enumerateDevices();
	if (N == 0) {
		exit(2);
	};
	printf("Found devices: %d \n", freenect2.enumerateDevices());
	for (int i = 0; i < N; ++i) {
		std::shared_ptr<libfreenect2::Freenect2Device> d(freenect2.openDevice(i));

		if (!d) exit(3);

		//============================================================================================
		//retrieve built-in parameters
		//============================================================================================

		d->start();
		const unsigned venId = d->VendorId;
		std::string serial = d->getSerialNumber();
		libfreenect2::Freenect2Device::ColorCameraParams colorCamParams = d->getColorCameraParams();
		libfreenect2::Freenect2Device::IrCameraParams irCamParams = d->getIrCameraParams();
		d->stop();

		//============================================================================================
		//print them to standard output
		//============================================================================================

		printf("VendorID: %d \n", venId);
		printf("Serial Number: %s \n", serial.c_str());

		printf("\nRGB Camera Parameters: \n");
		printf("f_x: %f15 \n", colorCamParams.fx);
		printf("f_y: %f15 \n", colorCamParams.fy);
		printf("c_x: %f15 \n", colorCamParams.cx);
		printf("c_y: %f15 \n", colorCamParams.cy);
		printf("shift_d: %f15 \n", colorCamParams.shift_d);
		printf("shift_m: %f15 \n", colorCamParams.shift_m);

		printf("mx_x0y0: %f15 \n", colorCamParams.mx_x0y0);
		printf("mx_x0y1: %f15 \n", colorCamParams.mx_x0y1);
		printf("mx_x0y2: %f15 \n", colorCamParams.mx_x0y2);
		printf("mx_x0y3: %f15 \n", colorCamParams.mx_x0y3);
		printf("mx_x1y0: %f15 \n", colorCamParams.mx_x1y0);
		printf("mx_x1y1: %f15 \n", colorCamParams.mx_x1y1);
		printf("mx_x1y2: %f15 \n", colorCamParams.mx_x1y2);
		printf("mx_x2y0: %f15 \n", colorCamParams.mx_x2y0);
		printf("mx_x2y1: %f15 \n", colorCamParams.mx_x2y1);
		printf("mx_x3y0: %f15 \n", colorCamParams.mx_x3y0);

		printf("my_x0y0: %f15 \n", colorCamParams.my_x0y0);
		printf("my_x0y1: %f15 \n", colorCamParams.my_x0y1);
		printf("my_x0y2: %f15 \n", colorCamParams.my_x0y2);
		printf("my_x0y3: %f15 \n", colorCamParams.my_x0y3);
		printf("my_x1y0: %f15 \n", colorCamParams.my_x1y0);
		printf("my_x1y1: %f15 \n", colorCamParams.my_x1y1);
		printf("my_x1y2: %f15 \n", colorCamParams.my_x1y2);
		printf("my_x2y0: %f15 \n", colorCamParams.my_x2y0);
		printf("my_x2y1: %f15 \n", colorCamParams.my_x2y1);
		printf("my_x3y0: %f15 \n", colorCamParams.my_x3y0);


		printf("\nIR Camera Parameters: \n");
		printf("f_x: %f15 \n", irCamParams.fx);
		printf("f_y: %f15 \n", irCamParams.fy);
		printf("c_x: %f15 \n", irCamParams.cx);
		printf("c_y: %f15 \n", irCamParams.cy);
		printf("k1: %f15 \n", irCamParams.k1);
		printf("k2: %f15 \n", irCamParams.k2);
		printf("k3: %f15 \n", irCamParams.k3);
		printf("p1: %f15 \n", irCamParams.p1);
		printf("p2: %f15 \n", irCamParams.p2);

		//============================================================================================
		//Generate OpenCV matrices
		//============================================================================================

		cv::Mat colorIntrinsics(3, 3, CV_32F);
		colorIntrinsics.at<float>(0, 0) = colorCamParams.fx;
		colorIntrinsics.at<float>(1, 1) = colorCamParams.fy;
		colorIntrinsics.at<float>(2, 0) = colorCamParams.cx;
		colorIntrinsics.at<float>(2, 1) = colorCamParams.cy;
		colorIntrinsics.at<float>(2, 2) = 1.0F;

		float rgbMomentsArr[2][10] = {
				{
						colorCamParams.mx_x0y0,
						colorCamParams.mx_x0y1,
						colorCamParams.mx_x0y2,
						colorCamParams.mx_x0y3,
						colorCamParams.mx_x1y0,
						colorCamParams.mx_x1y1,
						colorCamParams.mx_x1y2,
						colorCamParams.mx_x2y0,
						colorCamParams.mx_x2y1,
						colorCamParams.mx_x3y0 },
				{
						colorCamParams.my_x0y0,
						colorCamParams.my_x0y1,
						colorCamParams.my_x0y2,
						colorCamParams.my_x0y3,
						colorCamParams.my_x1y0,
						colorCamParams.my_x1y1,
						colorCamParams.my_x1y2,
						colorCamParams.my_x2y0,
						colorCamParams.my_x2y1,
						colorCamParams.my_x3y0 }
		};
		cv::Mat colorMoments(2, 10, CV_32F, &rgbMomentsArr);

		cv::Mat depthIntrinsics(3, 3, CV_32F);
		depthIntrinsics.at<float>(0, 0) = irCamParams.fx;
		depthIntrinsics.at<float>(1, 1) = irCamParams.fy;
		depthIntrinsics.at<float>(2, 0) = irCamParams.cx;
		depthIntrinsics.at<float>(2, 1) = irCamParams.cy;
		depthIntrinsics.at<float>(2, 2) = 1.0F;

		float depthDistCoeffsArr[5] = { irCamParams.k1, irCamParams.k2, irCamParams.k3, irCamParams.p1, irCamParams.p2 };

		cv::Mat depthDistCoeffs(1, 5, CV_32F, &depthDistCoeffsArr);

		//============================================================================================
		//print matrices out
		//============================================================================================

		std::cout << "RGB Camera intrinsics: " << std::endl << colorIntrinsics << std::endl;
		std::cout << "RGB Camera moments: " << std::endl << colorMoments << std::endl;
		std::cout << "Depth/IR Camera intrinsics: " << std::endl << depthIntrinsics << std::endl;
		std::cout << "Depth/IR Camera distortion coefficients: " << std::endl << depthDistCoeffs << std::endl;
	}
}

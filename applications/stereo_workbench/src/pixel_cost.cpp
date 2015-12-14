/*
 * pixel_cost.cpp
 *
 *  Created on: Dec 10, 2015
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
#include "pixel_cost.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/hal/intrin.hpp>

#include <reco/utils/debug_util.h>
#include <reco/utils/cpp_exception_util.h>

namespace reco{
namespace stereo_workbench{
using namespace cv;


void calculate_row_cost_L2( const Mat& img1, const Mat& img2, int y,
        int minD, int maxD, CostType* cost,
        PixType* buffer, const PixType* lookup_table,
        int tabOfs, int){
	int image_width = img1.cols; int channel_number = img1.channels();
	const int minX1 = std::max(maxD, 0), maxX1 = image_width + std::min(minD, 0);
	int D = maxD - minD;
	//under regular circumstances, from maxD up to image_width
	if(channel_number == 1){
		for( int x = minX1, xd=0; x < maxX1; x++,xd++){
			PixType px2 = img2.at<PixType>(y,x);
			for( int d = minD; d < maxD; d++ ){
				PixType px1 = img1.at<PixType>(y,x-d);
				cost[xd*D + d] = (CostType)(std::abs(px1 - px2));
			}
		}
	}else{
		for( int x = minX1, xd=0; x < maxX1; x++,xd++){
			cv::Vec3b px1 = img1.at<cv::Vec3b>(y,x);
			for( int d = minD; d < maxD; d++ ){
				cv::Vec3b px2 = img2.at<cv::Vec3b>(y,x-d);
				double norm = cv::norm(px1,px2,cv::NormTypes::NORM_L2);
				cost[xd*D + d] = (CostType)(norm*0.579614388);
			}
		}
	}
}
DAISY_stereo_cost_calculator::DAISY_stereo_cost_calculator(const cv::Mat& img1, const cv::Mat& img2,
			const semiglobal_matcher_parameters& params):
			abstract_stereo_cost_calculator(img1, img2){
	this->minD = params.minDisparity;
	this->maxD = params.minDisparity + params.numDisparities;
	this->image_width = img1.cols;
	Ptr<xfeatures2d::DAISY> daisy = cv::xfeatures2d::DAISY::create(15,3,8,8,cv::xfeatures2d::DAISY::NRM_PARTIAL,
			cv::noArray(),true,false);
	daisy->compute(img1,descriptors1);
	daisy->compute(img2,descriptors2);
};

static int max = 0;
void DAISY_stereo_cost_calculator::compute(int y,CostType* cost){
	const int minX1 = std::max(maxD, 0), maxX1 = image_width + std::min(minD, 0);
	int D = maxD - minD;
	const int y_offset = y*image_width;
	const double norm_factor = 100.0D;

	//#pragma omp parallel for
	for( int x = y_offset + minX1, xd = 0; x < y_offset + maxX1; x++, xd++ ){
		Mat desc1 = descriptors1.row(x);
		for( int d = minD; d < maxD; d++ ){
			Mat desc2 = descriptors2.row(x - d);
			double nrm = cv::norm(desc1,desc2,cv::NormTypes::NORM_L2SQR);
//			if((int)(nrm*norm_factor) > max){
//				max = nrm*norm_factor;
//				dpt(max);
//			}
			cost[xd*D + d] = (CostType)(nrm * norm_factor);
		}
	}
}

BT_stereo_cost_calculator::BT_stereo_cost_calculator(const cv::Mat& img1, const cv::Mat& img2, const semiglobal_matcher_parameters& params):
				abstract_stereo_cost_calculator(img1, img2){
	this->minD = params.minDisparity;
	this->maxD = params.minDisparity + params.numDisparities;
	Mat buffer;
	int tmpBufSize   = img1.cols*16*img1.channels()*sizeof(PixType);
	mat_buf.create(1, tmpBufSize, CV_8U);

	this->buffer = (PixType*)mat_buf.ptr();
	table_offset = 256*4;
	int TAB_SIZE = 256 + table_offset*2;
	ftzero = std::max(params.preFilterCap, 15) | 1;
	clip_table = new PixType[TAB_SIZE];
	clip_table_relevant = clip_table + table_offset;

	int ftzero = std::max(params.preFilterCap, 15) | 1;
	//value at TAB_OFS = ftzero
	//values beyond that grow for ftzero up to 2*ftzero, then keep at 2*ftzero
	for(int k = 0; k < TAB_SIZE; k++ )
		clip_table[k] = (PixType)(std::min(std::max(k - table_offset, -ftzero), ftzero) + ftzero);
}



BT_stereo_cost_calculator::~BT_stereo_cost_calculator(){
	if(clip_table){
		delete clip_table;
	}
}
void BT_stereo_cost_calculator::compute(int y, CostType* cost){
	int x, c, image_width = img1.cols, channel_number = img1.channels();
	int minX1 = std::max(maxD, 0), maxX1 = image_width + std::min(minD, 0);
	//under regular circumstances, 0		//under regular circumstances, image_width
	int minX2 = std::max(minX1 - maxD, 0), maxX2 = std::min(maxX1 - minD, image_width);
	int D = maxD - minD;
	int width1 = maxX1 - minX1; //under regular circumstances, image_width - maxD
	int width2 = maxX2 - minX2; //under regular circumstances, image_width

	const PixType* row_img1 = img1.ptr<PixType>(y);
	const PixType* row_img2 = img2.ptr<PixType>(y);

	//TODO why is this offset necessary?
	PixType* precomputed_buffer_img1 = buffer + width2*2;
	PixType* precomputed_buffer_img2 = precomputed_buffer_img1 + image_width*channel_number*2;

	for( c = 0; c < channel_number*2; c++ ){
		precomputed_buffer_img1[image_width*c] = precomputed_buffer_img1[image_width*c + image_width-1] =
		precomputed_buffer_img2[image_width*c] = precomputed_buffer_img2[image_width*c + image_width-1] = clip_table[0];
	}

	//TODO: aren't prev1 == prev2 and next1 == next2, since images always have the same dimensions and,
	//therefore, step?
	int prev1 = y > 0 ? -(int)img1.step : 0;
	int prev2 = y > 0 ? -(int)img2.step : 0;
	int next1 = y < img1.rows-1 ? (int)img1.step : 0;
	int next2 = y < img2.rows-1 ? (int)img2.step : 0;

	if( channel_number == 1 ){
		for( x = 1; x < image_width-1; x++ ){
			//image 1
			precomputed_buffer_img1[x] =
					clip_table_relevant[(row_img1[x+1] - row_img1[x-1])*2 + //current pixel surround
								  row_img1[x+prev1+1] - row_img1[x+prev1-1] + //above pixel surround
								  row_img1[x+next1+1] - row_img1[x+next1-1]]; //below pixel surround

			precomputed_buffer_img1[x+image_width] = row_img1[x]; //current pixel value
			//image 2
			//fill from rightmost postion to left
			precomputed_buffer_img2[image_width-1-x] =
					clip_table_relevant[(row_img2[x+1] - row_img2[x-1])*2 +
								  row_img2[x+prev2+1] - row_img2[x+prev2-1] +
								  row_img2[x+next2+1] - row_img2[x+next2-1]];
			precomputed_buffer_img2[image_width-1-x+image_width] = row_img2[x]; //current pixel value
		}
	}else{
		//assume 3-channel image
		for( x = 1; x < image_width-1; x++ ){

			//image 1
			precomputed_buffer_img1[x] = //ch1
					clip_table_relevant[(row_img1[x*3+3] - row_img1[x*3-3])*2 +
								 row_img1[x*3+prev1+3] - row_img1[x*3+prev1-3] +
								 row_img1[x*3+next1+3] - row_img1[x*3+next1-3]];
			precomputed_buffer_img1[x+image_width] = //ch2
					clip_table_relevant[(row_img1[x*3+4] - row_img1[x*3-2])*2 +
								 row_img1[x*3+prev1+4] - row_img1[x*3+prev1-2] +
								 row_img1[x*3+next1+4] - row_img1[x*3+next1-2]];
			precomputed_buffer_img1[x+image_width*2] = //ch3
					clip_table_relevant[(row_img1[x*3+5] - row_img1[x*3-1])*2 +
								 row_img1[x*3+prev1+5] - row_img1[x*3+prev1-1] +
								 row_img1[x*3+next1+5] - row_img1[x*3+next1-1]];

			precomputed_buffer_img1[x+image_width*3] = row_img1[x*3];
			precomputed_buffer_img1[x+image_width*4] = row_img1[x*3+1];
			precomputed_buffer_img1[x+image_width*5] = row_img1[x*3+2];

			//image2
			precomputed_buffer_img2[image_width-1-x] =
					clip_table_relevant[(row_img2[x*3+3] - row_img2[x*3-3])*2 + row_img2[x*3+prev2+3] -
								 row_img2[x*3+prev2-3] + row_img2[x*3+next2+3] - row_img2[x*3+next2-3]];
			precomputed_buffer_img2[image_width-1-x+image_width] =
					clip_table_relevant[(row_img2[x*3+4] - row_img2[x*3-2])*2 + row_img2[x*3+prev2+4] -
								 row_img2[x*3+prev2-2] + row_img2[x*3+next2+4] - row_img2[x*3+next2-2]];
			precomputed_buffer_img2[image_width-1-x+image_width*2] =
					clip_table_relevant[(row_img2[x*3+5] - row_img2[x*3-1])*2 + row_img2[x*3+prev2+5] -
								 row_img2[x*3+prev2-1] + row_img2[x*3+next2+5] - row_img2[x*3+next2-1]];
			precomputed_buffer_img2[image_width-1-x+image_width*3] = row_img2[x*3];
			precomputed_buffer_img2[image_width-1-x+image_width*4] = row_img2[x*3+1];
			precomputed_buffer_img2[image_width-1-x+image_width*5] = row_img2[x*3+2];
		}
	}

	memset( cost, 0, width1*D*sizeof(cost[0]) );

	buffer -= minX2;
	cost -= minX1*D + minD; // simplify the cost indices inside the loop

	//for each channel of each image
	//v refers to img2, u refers to img1
	for( c = 0; c < channel_number*2; c++,
		//go to next buffer row
		precomputed_buffer_img1 += image_width, precomputed_buffer_img2 += image_width ){
		int diff_scale = c < channel_number ? 0 : 2;

		// precompute
		//   v0 = min(row2[x-1/2], row2[x], row2[x+1/2]) and
		//   v1 = max(row2[x-1/2], row2[x], row2[x+1/2]) and
		//normally, traverses whole image_width
		for( x = minX2; x < maxX2; x++ ){
			int v = precomputed_buffer_img2[x];//this is in reverse order
			int vl = x > 0 ? (v + precomputed_buffer_img2[x-1])/2 : v;
			int vr = x < image_width-1 ? (v + precomputed_buffer_img2[x+1])/2 : v;
			int v0 = std::min(vl, vr); v0 = std::min(v0, v);
			int v1 = std::max(vl, vr); v1 = std::max(v1, v);
			buffer[x] = (PixType)v0;
			buffer[x + width2] = (PixType)v1;
		}

		//under regular circumstances, from maxD up to image_width
		for( x = minX1; x < maxX1; x++ ){
			int u = precomputed_buffer_img1[x];
			int ul = x > 0 ? (u + precomputed_buffer_img1[x-1])/2 : u;
			int ur = x < image_width-1 ? (u + precomputed_buffer_img1[x+1])/2 : u;
			int u0 = std::min(ul, ur); u0 = std::min(u0, u);
			int u1 = std::max(ul, ur); u1 = std::max(u1, u);
		#if CV_SIMD128
			v_uint8x16 _u  = v_setall_u8((uchar)u), _u0 = v_setall_u8((uchar)u0);
			v_uint8x16 _u1 = v_setall_u8((uchar)u1);

			for( int d = minD; d < maxD; d += 16 ){
				v_uint8x16 _v  = v_load(precomputed_buffer_img2  + image_width-x-1 + d);
				v_uint8x16 _v0 = v_load(buffer + image_width-x-1 + d);
				v_uint8x16 _v1 = v_load(buffer + image_width-x-1 + d + width2);
				v_uint8x16 c0 = v_max(_u - _v1, _v0 - _u);
				v_uint8x16 c1 = v_max(_v - _u1, _u0 - _v);
				v_uint8x16 diff = v_min(c0, c1);

				v_int16x8 _c0 = v_load_aligned(cost + x*D + d);
				v_int16x8 _c1 = v_load_aligned(cost + x*D + d + 8);

				v_uint16x8 diff1,diff2;
				v_expand(diff,diff1,diff2);
				v_store_aligned(cost + x*D + d,     _c0 + v_reinterpret_as_s16(diff1 >> diff_scale));
				v_store_aligned(cost + x*D + d + 8, _c1 + v_reinterpret_as_s16(diff2 >> diff_scale));
			}
		#else
			for( int d = minD; d < maxD; d++ ){

				int v = precomputed_buffer_img2[image_width-x-1 + d];
				int v0 = buffer[image_width-x-1 + d];
				int v1 = buffer[image_width-x-1 + d + width2];
				int c0 = std::max(0, u - v1); c0 = std::max(c0, v0 - u);
				int c1 = std::max(0, v - u1); c1 = std::max(c1, u0 - v);
				CostType new_cost = (CostType)(cost[x*D+d] + (std::min(c0, c1) >> diff_scale));
				cost[x*D + d] = new_cost;
			}
		#endif
		}
	}
}

std::unique_ptr<abstract_stereo_cost_calculator> build_stereo_cost_calculator(pixel_cost_type type,
		const cv::Mat& img1, const cv::Mat& img2,const semiglobal_matcher_parameters& params){
	switch(type){
	case BIRCHFIELD_TOMASI:
		return std::unique_ptr<abstract_stereo_cost_calculator>(new BT_stereo_cost_calculator(img1,img2,params));
		break;
	case DAISY:
		return std::unique_ptr<abstract_stereo_cost_calculator>(new DAISY_stereo_cost_calculator(img1,img2,params));
		break;
	default:
		err2(std::runtime_error, "Unknown semiglobal matcher cost type: " << static_cast<int>(type));
		return std::unique_ptr<abstract_stereo_cost_calculator>();
		break;
	}
}

}//stereo_workbench
}//reco

/*
 * opencl_filter_manager.h
 *
 *  Created on: Dec 4, 2015
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

#pragma once

#include <opencv2/core.hpp>
#include <CL/cl.hpp>

namespace reco {
namespace stereo_workbench {


class opencl_filter_manager {
public:
	opencl_filter_manager( cv::Size image_size,
			cv::Size cell_size = cv::Size(),
			int num_channels = 3,
			std::string platform_keyword = "NVIDIA",
			cl_device_type device_type = CL_DEVICE_TYPE_ALL,
			std::string device_keyword = "",
			int device_index = 0);
	virtual ~opencl_filter_manager();

	//TODO:
	void traverse_image(cv::Mat image);

private:
	static cl::Device select_device(std::string platform_keyword = "NVIDIA",
			cl_device_type device_type = CL_DEVICE_TYPE_ALL, std::string device_keyword = "", int device_index = 0);
	static int estimate_num_processors_per_SM(cl::Device device);
	static void determine_image_types(cl::Context context, cl::ImageFormat& uint8_format,
			cl::ImageFormat& int16_format,cl::ImageFormat& uint32_format);
	static int estimate_available_cl_device_memory(cl::Device device);
	static bool have_opengl_extension(std::string extension);
	static int determine_warp_size(cl::Device device, cl::Context context);

	static const std::vector<int> nvidia_vendor_ids;
	static const std::vector<int> amd_vendor_ids;

	cl::Device device;
	cl::Context context;
	cl::CommandQueue queue;
	cl::ImageFormat uint8_format;
	cl::ImageFormat int16_format;
	cl::ImageFormat uint32_format;
	cv::Size cell_size;
	cv::Size image_size;
	cv::Size cells_per_image;
	cv::Size group_dims;
	cv::Size block_dims;

	int warp_size;
	int max_warps;
	int half_warp_size;
	int max_threads;
	int num_channels;
	int schedule_optimized_n_warps;
	int input_stride;


};

} /* namespace stereo_workbench */
} /* namespace reco */

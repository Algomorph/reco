/*
 * opencl_filter_manager.cpp
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

#include <src/opencl_filter_manager.h>
#include <reco/utils/cpp_exception_util.h>
#include <GL/glew.h>
#include <GL/glut.h>


#include <thread>
#include <regex>


namespace reco {
namespace stereo_workbench {

#ifndef GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX
#define GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX 0x9049
#endif

#ifndef GL_VBO_FREE_MEMORY_ATI
#define GL_VBO_FREE_MEMORY_ATI 0x87FB
#endif

const std::vector<int> opencl_filter_manager::nvidia_vendor_ids({4318});
const std::vector<int> opencl_filter_manager::amd_vendor_ids({4130,4098});

opencl_filter_manager::opencl_filter_manager(
		cv::Size image_size, cv::Size cell_size, int num_channels,
		std::string platform_keyword,
		cl_device_type device_type, std::string device_keyword, int device_index) :
				image_size(image_size),
				cell_size(cell_size),
				num_channels(num_channels){

	this->device = opencl_filter_manager::select_device(platform_keyword, device_type,
			device_keyword, device_index);

	device_type = device.getInfo<CL_DEVICE_TYPE>();

	float avail_memory_divisor;

	switch(device_type){
	case CL_DEVICE_TYPE_CPU:
		//we have lots of memory, but we probably need a good portion of it for other applications
		avail_memory_divisor = 4.0F;
		break;
	case CL_DEVICE_TYPE_GPU:
		//we don't have as much memory, but we'll only need about a third for all other processes
		avail_memory_divisor = 1.5;
		break;
	default:
		//arbitrary accelerator -- use 1/3, adjust as necessary
		avail_memory_divisor = 3;
		break;
	}

	context = cl::Context(device);
#ifdef NEED_GLOBAL_MEM
	int global_mem = device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>();
	int local_mem = device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>();
	int work_group_size = device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>();
#endif
	//number of streaming multiprocessors
	int num_SMs = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
	max_threads = opencl_filter_manager::estimate_num_processors_per_SM(device) * num_SMs;
	opencl_filter_manager::determine_image_types(context, uint8_format, int16_format, uint32_format);

	warp_size = determine_warp_size(device,context);
	half_warp_size = warp_size >> 1;
	int avail_mem = estimate_available_cl_device_memory(device);
	const int channel_num = 4;

	if(cell_size.height == 0 && cell_size.width == 0){

		int memory_bound = std::pow(2,int(std::log2(std::sqrt(avail_mem / avail_memory_divisor / channel_num))));
		int max_width = device.getInfo<CL_DEVICE_IMAGE2D_MAX_WIDTH>();
		int max_height = device.getInfo<CL_DEVICE_IMAGE2D_MAX_HEIGHT>();
		int cell_lateral_size = std::min({memory_bound, max_width, max_height});
		cell_size.height = cell_size.width = cell_lateral_size;
	}

	cells_per_image = cv::Size(image_size.width / cell_size.width, image_size.height / cell_size.height);
	group_dims = cv::Size(
			(cell_size.width + max_threads - 1) / max_threads,
			(cell_size.height + max_threads - 1) / max_threads);
	block_dims = cv::Size(cell_size.height, cell_size.width);
	max_warps = max_threads / warp_size;
	schedule_optimized_n_warps = max_warps - 1;
	input_stride = cell_size.width * schedule_optimized_n_warps;
	queue = cl::CommandQueue(context, device);
}



opencl_filter_manager::~opencl_filter_manager(){

}

cl::Device opencl_filter_manager::select_device(std::string platform_keyword,
		cl_device_type device_type, std::string device_keyword, int device_index){
	std::vector<cl::Platform> all_platforms;
	cl::Platform::get(&all_platforms);
	if(all_platforms.size() == 0){
		err2(std::runtime_error, "No OpenCL platforms detected");
	}
	cl::Platform selected_platform;
	bool platform_found = false;
	std::regex platform_keyword_regex(platform_keyword, std::regex_constants::icase);
	for(cl::Platform platform : all_platforms){
		std::string platform_name = platform.getInfo<CL_PLATFORM_NAME>();
		if(std::regex_search(platform_name, platform_keyword_regex)){
			selected_platform = platform;
			platform_found = true;
			break;
		}
	}
	if(!platform_found){
		err2(std::runtime_error, "No OpenCL platform found matching keyword \"" <<
						platform_keyword << "\"");
	}
	std::vector<cl::Device> devices;
	selected_platform.getDevices(device_type,&devices);
	if(devices.size()  == 0){
		err2(std::runtime_error, "No OpenCL device on platform " <<
				selected_platform.getInfo<CL_PLATFORM_NAME>());
	}

	if(device_keyword.length() > 0){
		std::regex device_keyword_regex(device_keyword, std::regex_constants::icase);
		for(cl::Device device : devices){
			std::string device_name = device.getInfo<CL_DEVICE_NAME>();
			if(std::regex_search(device_name, device_keyword_regex)){
				return device;
			}
		}
	}else{
		if(device_index > (int)devices.size()){
			err2(std::runtime_error, "OpenCL device index out of range. Got " << device_index
					<< ", acceptable values should be in [0, " << devices.size() << "] for platform "
					<< selected_platform.getInfo<CL_PLATFORM_NAME>());
		}else{
			return devices[device_index];
		}
	}
}

bool opencl_filter_manager::have_opengl_extension(std::string extension){
	int win = glutCreateWindow("test");
	GLint num_extensions = 0;
	bool found = false;
	glGetIntegerv( GL_NUM_EXTENSIONS, &num_extensions);
	for (int i = 0; i < num_extensions && !found; ++i) {
		std::string extension_string = std::string((const char*) glGetStringi( GL_EXTENSIONS, i));
		if(extension_string == extension){
			found = true;
		}
	}
	glutDestroyWindow(win);
	return found;
}

int opencl_filter_manager::estimate_num_processors_per_SM(cl::Device device){
	switch(device.getInfo<CL_DEVICE_TYPE>()){
	case CL_DEVICE_TYPE_CPU:
		{
			int num_SMs = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
			return std::thread::hardware_concurrency() / num_SMs;
		}
	case CL_DEVICE_TYPE_GPU:
		{
			int vendor_id = device.getInfo<CL_DEVICE_VENDOR_ID>();
			if(std::find(nvidia_vendor_ids.begin(),nvidia_vendor_ids.end(), vendor_id) !=
					nvidia_vendor_ids.end()){
				return 8;
			}else if(std::find(amd_vendor_ids.begin(),amd_vendor_ids.end(), vendor_id) !=
					amd_vendor_ids.end()){
				return 4;
			}
		}
		return 1;
	default:
		return 1;
	}
}

void opencl_filter_manager::determine_image_types(cl::Context context, cl::ImageFormat& uint8_format,
			cl::ImageFormat& int16_format,cl::ImageFormat& uint32_format){
	std::vector<cl::ImageFormat> formats;
	context.getSupportedImageFormats(CL_MEM_READ_WRITE, CL_MEM_OBJECT_IMAGE2D,&formats);
	bool uint8_picked, int16_picked, uint32_picked;
	uint8_picked = int16_picked = uint32_picked = false;

	for (cl::ImageFormat format : formats){
		cl_channel_type channel_type = format.image_channel_data_type;
		cl_channel_order channel_order = format.image_channel_order;
		switch(channel_type){
		case CL_UNSIGNED_INT8:
			if(channel_order == CL_RGB){
				uint8_format = format; //ideal case;
				uint8_picked = true;
			}else if(!uint8_picked && channel_order == CL_RGBA){
				uint8_format = format; //second best
				uint8_picked = true;
			}
			break;
		case CL_SIGNED_INT16:
			if(channel_order == CL_RGB){
				int16_format = format; //ideal case;
				int16_picked = true;
			}else if(!int16_picked && channel_order == CL_RGBA){
				int16_format = format; //second best
				int16_picked = true;
			}
			break;
		case CL_UNSIGNED_INT32:
			if(channel_order == CL_RGB){
				uint32_format = format; //ideal case;
				uint32_picked = true;
			}else if(!uint32_picked && channel_order == CL_RGBA){
				uint32_format = format; //second best
				uint32_picked = true;
			}
			break;
		}

	}
}

int opencl_filter_manager::estimate_available_cl_device_memory(cl::Device device){
	int mb_estimated = 500;
	int b_in_mb = 1048576; //2^20
	int b_in_kb = 1024; //2^10
	int bytes_estimated = b_in_mb * mb_estimated;
	bool success = false;

	switch(device.getInfo<CL_DEVICE_TYPE>()){
	case CL_DEVICE_TYPE_CPU:
		{
			//assume we are unbounded or, say, 1 GB
			bytes_estimated = 1073741824;
		}
		break;
	case CL_DEVICE_TYPE_GPU:
		{
			int vendor_id = device.getInfo<CL_DEVICE_VENDOR_ID>();
			if(std::find(nvidia_vendor_ids.begin(),nvidia_vendor_ids.end(), vendor_id) !=
					nvidia_vendor_ids.end() && have_opengl_extension("GL_NVX_gpu_memory_info")){
				int win = glutCreateWindow("test");
				GLint available_memory_in_kb = 0;
				glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX,
									   &available_memory_in_kb);
				if(available_memory_in_kb > 0){
					bytes_estimated = b_in_kb * available_memory_in_kb;
				}
				glutDestroyWindow(win);
			}else if(std::find(amd_vendor_ids.begin(),amd_vendor_ids.end(), vendor_id) !=
					amd_vendor_ids.end() && have_opengl_extension("GL_ATI_meminfo")){
				GLint bytes_estimated = 0;
				glGetIntegerv(GL_VBO_FREE_MEMORY_ATI, &bytes_estimated);
			}
		}
		break;
	default:
		break;
	}


	return bytes_estimated;
}

int opencl_filter_manager::determine_warp_size(cl::Device device, cl::Context context){
	std::string test_kernel_code =
			"__kernel \n"
			"void testKernel(){}";
	cl::Program::Sources sources;
	sources.push_back({test_kernel_code.c_str(), test_kernel_code.length()});
	cl::Program program(context,sources);
	if(program.build({device})!= CL_SUCCESS){
		err2(std::runtime_error," Error building test kernel: "
				<< program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << "\n");
	}
	std::vector<cl::Kernel> kernels;
	program.createKernels(&kernels);
	cl::Kernel test_kernel = kernels[0];
	test_kernel.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device);

}


} /* namespace stereo_workbench */
} /* namespace reco */

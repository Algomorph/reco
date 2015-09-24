/*
 * kinect2_pipe.h
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_KINECT2_PIPE_H_
#define RECO_DATAPIPE_KINECT2_PIPE_H_

//datapipe
#include <reco/datapipe/multifeed_pipe.h>
#include <reco/datapipe/kinect_v2_info.h>

namespace reco{
namespace datapipe{

class kinect2_pipe : public multifeed_pipe<kinect_v2_info::channels.size(),kinect_v2_info::channels>{

public:
	enum kinect2_data_source {
		hal_log, kinect2_device, image_folder
	};
	kinect2_pipe(multichannel_pipe::buffer_type buffer,kinect2_data_source source = hal_log,
			const std::string& path = "capture.log");
	virtual ~kinect2_pipe();
	int get_num_kinects();

protected:


private:
	kinect2_data_source source;
	std::string path;
	static std::string compile_camera_uri(kinect2_data_source source, std::string path);

};

}//end namespace datapipe
}//end namespace reco

#endif /* RECO_DATAPIPE_KINECT2_PIPE_H_ */

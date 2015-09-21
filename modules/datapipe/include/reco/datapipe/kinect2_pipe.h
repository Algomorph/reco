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
	kinect2_pipe(multichannel_pipe::buffer_type buffer);
	virtual ~kinect2_pipe();


};

}//end namespace datapipe
}//end namespace reco

#endif /* RECO_DATAPIPE_KINECT2_PIPE_H_ */

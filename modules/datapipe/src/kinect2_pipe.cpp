/*
 * kinect2_pipe.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/kinect2_pipe.h>

namespace reco{
namespace datapipe{

kinect2_pipe::kinect2_pipe(multichannel_pipe::buffer_type buffer):multifeed_pipe(buffer){

}

kinect2_pipe::~kinect2_pipe(){}

}//end namespace datapipe
}//end namespace reco



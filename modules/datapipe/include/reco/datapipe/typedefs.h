/*
 * typedefs.h
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_TYPEDEFS_H_
#define RECO_DATAPIPE_TYPEDEFS_H_

//HAL

#include <HAL/Messages/ImageArray.h>
//utils
#include <reco/utils/queue.h>
//standard
#include <memory>

namespace reco{
namespace datapipe{
typedef std::shared_ptr<utils::queue<std::shared_ptr<hal::ImageArray>>> frame_buffer_type;

}//end namespace reco
}//end namespace datapipe





#endif /* RECO_DATAPIPE_TYPEDEFS_H_ */

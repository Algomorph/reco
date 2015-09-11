/*
 * reconstructor.h
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_WORKBENCH_RECONSTRUCTOR_H_
#define RECO_WORKBENCH_RECONSTRUCTOR_H_

//standard
#include <memory>

//utils
#include <reco/utils/worker.h>
#include <reco/utils/queue.h>

//HAL
#include <HAL/Messages/ImageArray.h>

//local
#include "point_cloud_buffer.h"

namespace reco {
namespace workbench {



/**
 * @brief Responsible for processing the range images and coming up with 3D meshes reconstructed from them.
 */
class reconstructor: public utils::worker {
public:
	/**
	 * Type of the input buffer required by the reconstructor
	 */
	typedef std::shared_ptr<utils::unbounded_queue<std::shared_ptr<hal::ImageArray>>> input_buffer_type;
private:
	std::shared_ptr<point_cloud_buffer> output_buffer;
	input_buffer_type input_buffer;
protected:
	virtual bool do_unit_of_work();

public:
	reconstructor(std::shared_ptr<point_cloud_buffer> result_buffer);
	virtual ~reconstructor();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_RECONSTRUCTOR_H_ */

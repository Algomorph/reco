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

//local
#include "point_cloud_buffer.h"

namespace reco {
namespace workbench {
/**
 * @brief Responsible for processing the range images and coming up with 3D meshes reconstructed from them.
 */
class reconstructor: public utils::worker {
private:
	std::shared_ptr<point_cloud_buffer> result_buffer;

protected:
	virtual bool do_unit_of_work();

public:
	reconstructor(std::shared_ptr<point_cloud_buffer> result_buffer);
	virtual ~reconstructor();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_RECONSTRUCTOR_H_ */

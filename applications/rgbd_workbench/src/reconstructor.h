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
#include "calibration_parameters.h"
#include "point_cloud_buffer.h"

//datapipe
#include <reco/datapipe/typedefs.h>

namespace reco {
namespace rgbd_workbench {

/**
 * @brief Responsible for processing the range images and coming up with 3D meshes reconstructed from them.
 */
class reconstructor:
		public QObject,
		public utils::worker
		 {
Q_OBJECT

private:

	std::shared_ptr<point_cloud_buffer> output_buffer;
	std::shared_ptr<calibration_parameters> calibration;
	datapipe::frame_buffer_type input_buffer;

	std::vector<uint32_t> cloud_colors;

protected:
	virtual bool do_unit_of_work();
	virtual void pre_thread_join();

public:
	reconstructor(datapipe::frame_buffer_type input_buffer,
			std::shared_ptr<point_cloud_buffer> output_buffer,
			std::shared_ptr<calibration_parameters> calibration);
	virtual ~reconstructor();

signals:
	void frame_consumed();
	void frame_processed();

};
}/* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_RECONSTRUCTOR_H_ */

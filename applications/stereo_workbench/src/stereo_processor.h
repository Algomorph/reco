/*
 * stereo_processor.h
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */
//TODO: 750 remove header guards globally
#pragma once

//qt
#include <QObject>
//datapipe
#include <reco/datapipe/typedefs.h>
//utils
#include <reco/utils/worker.h>
//opencv
#include <opencv2/calib3d/calib3d.hpp>
//misc
#include <reco/misc/calibration_parameters.h>

#define USE_STEREO_SGBM

namespace reco {
namespace stereo_workbench {

class stereo_processor: public QObject, public utils::worker {

Q_OBJECT
public:
	stereo_processor(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<misc::calibration_parameters> calibration);
	virtual ~stereo_processor();
protected:
	virtual bool do_unit_of_work();
	virtual void pre_thread_join();
private:
	datapipe::frame_buffer_type input_frame_buffer;
	datapipe::frame_buffer_type output_frame_buffer;
#ifdef USE_STEREO_SGBM
	cv::StereoSGBM stereo_matcher;
#else
	cv::StereoBM stereo_matcher;
#endif

	std::shared_ptr<misc::calibration_parameters> calibration;

	calibu::LookupTable left_lut;
	calibu::LookupTable right_lut;

signals:
	void frame(std::shared_ptr<std::vector<cv::Mat>> images);

};

} /* namespace stereo_workbench */
} /* namespace reco */

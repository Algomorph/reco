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
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/xfeatures2d/xfeatures2d.hpp>

//calibu
#include <calibu/Calibu.h>
//std
#include <mutex>
#include "rectifier.h"

namespace reco {
namespace stereo_workbench {

class stereo_tuner: public QObject, public utils::worker {

Q_OBJECT
public:
	stereo_tuner(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<rectifier>  rectifier_instance = std::shared_ptr<rectifier>());
	virtual ~stereo_tuner();

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)
	cv::StereoSGBM stereo_matcher;
#elif CV_VERSION_MAJOR == 3
	cv::Ptr<cv::StereoSGBM> stereo_matcher;
#endif


	int get_v_offset();
	bool is_rectification_enabled();
	void set_rectifier(std::shared_ptr<rectifier> _rectifier);
	void toggle_rectification();

protected:
	virtual bool do_unit_of_work();
	virtual void pre_thread_join();
private:
	std::mutex rectify_guard;
	bool rectification_enabled;
	datapipe::frame_buffer_type input_frame_buffer;
	datapipe::frame_buffer_type output_frame_buffer;
	bool worker_shutting_down;
	int right_v_offset = 0;

	cv::Mat last_left;
	cv::Mat last_right;
	cv::Mat last_left_rectified;
	cv::Mat last_right_rectified;
	std::shared_ptr<rectifier> _rectifier;

	void recompute_disparity_if_paused();
	void recompute_disparity();
	void compute_disparity(cv::Mat left, cv::Mat right);

#if CV_VERSION_MAJOR == 3
	//TODO 203
	void compute_disparity_daisy(cv::Mat left, cv::Mat right);
#endif

public slots:
	void set_minimum_disparity(int value);
	void set_num_disparities(int value);
	void set_window_size(int value);
	void set_p1(int value);
	void set_p2(int value);
	void set_pre_filter_cap(int value);
	void set_uniqueness_ratio(int value);
	void set_speckle_window_size(int value);
	void set_speckle_range(int value);
	void set_v_offset(int value);
	void save_current();


signals:
	void frame(std::shared_ptr<std::vector<cv::Mat>> images);

};

} /* namespace stereo_workbench */
} /* namespace reco */

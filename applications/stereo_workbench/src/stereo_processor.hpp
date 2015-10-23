/*
 * stereo_processor.h
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

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
#include "rectifier.hpp"

namespace reco {
namespace stereo_workbench {

class stereo_processor_qt_base: public QObject{
	Q_OBJECT
	public slots:
		//tuning slots
		virtual void set_block_size(int value) = 0;
		virtual void set_disparity_max_diff(int value) = 0;
		virtual void set_minimum_disparity(int value) = 0;
		virtual void set_num_disparities(int value) = 0;
		virtual void set_speckle_range(int value) = 0;
		virtual void set_speckle_window_size(int value) = 0;
		virtual void set_v_offset(int value) = 0;

		//for diagnostics
		virtual void save_current_matcher_input() = 0;

	signals:
		void frame(std::shared_ptr<std::vector<cv::Mat>> images);
};


template<class MATCHER>
class stereo_processor: public stereo_processor_qt_base, public utils::worker {

	static_assert(
        std::is_base_of<cv::StereoMatcher, MATCHER>::value,
        "MATCHER must be a descendant of cv::StereoMatcher"
    );

public:
	stereo_processor(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<rectifier>  rectifier_instance = std::shared_ptr<rectifier>());
	virtual ~stereo_processor();


	int get_v_offset();
	bool is_rectification_enabled();
	void set_rectifier(std::shared_ptr<rectifier> _rectifier);
	void toggle_rectification();
	cv::Ptr<MATCHER> stereo_matcher;

protected:
	virtual bool do_unit_of_work();
	virtual void pre_thread_join();
	void recompute_disparity_if_paused();

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


	void recompute_disparity();
	void compute_disparity(cv::Mat left, cv::Mat right);

#if CV_VERSION_MAJOR == 3
	//TODO 203
	void compute_disparity_daisy(cv::Mat left, cv::Mat right);
#endif

public:

	//tuning slots
	virtual void set_block_size(int value);
	virtual void set_disparity_max_diff(int value);
	virtual void set_minimum_disparity(int value);
	virtual void set_num_disparities(int value);
	virtual void set_speckle_range(int value);
	virtual void set_speckle_window_size(int value);
	virtual void set_v_offset(int value);

	//slots for diagnostics
	virtual void save_current_matcher_input();
};

} /* namespace stereo_workbench */
} /* namespace reco */

#include "stereo_processor.tpp"

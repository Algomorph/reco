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

//calibu
#include <calibu/Calibu.h>
//std
#include <mutex>

#include <reco/stereo/rectifier.hpp>
#include <reco/stereo_tuner/matcher_qt_wrapper.hpp>

namespace reco {
namespace stereo_tuner {


class stereo_processor: public QObject, public utils::worker {
	Q_OBJECT

public:

	stereo_processor(datapipe::frame_buffer_type input_frame_buffer,
			datapipe::frame_buffer_type output_frame_buffer,
			std::shared_ptr<matcher_qt_wrapper_base> matcher = std::shared_ptr<matcher_qt_wrapper_base>(),
			std::shared_ptr<stereo::rectifier>  rectifier_instance = std::shared_ptr<stereo::rectifier>());
	virtual ~stereo_processor();
	bool is_rectification_enabled() const;
	void set_rectifier(std::shared_ptr<stereo::rectifier> _rectifier);
	void set_matcher(std::shared_ptr<matcher_qt_wrapper_base> matcher);
	void toggle_rectification();
	int get_v_offset() const;
	std::shared_ptr<matcher_qt_wrapper_base> get_matcher() const;


protected:

	std::shared_ptr<matcher_qt_wrapper_base> matcher;

	virtual bool do_unit_of_work();
	virtual void pre_thread_join();

private:
	std::mutex rectify_guard;
	std::mutex input_guard;
	bool rectification_enabled;
	datapipe::frame_buffer_type input_frame_buffer;
	datapipe::frame_buffer_type output_frame_buffer;
	bool worker_shutting_down;
	int right_v_offset = 0;

	cv::Mat last_left;
	cv::Mat last_right;
	cv::Mat last_left_rectified;
	cv::Mat last_right_rectified;
	cv::Mat disparity_normalized;
	std::shared_ptr<stereo::rectifier> _rectifier;

	void compute_disparity(cv::Mat left, cv::Mat right);

public slots:


	//tuning slot
	virtual void set_v_offset(int value);

	//for diagnostics
	virtual void save_current_matcher_input();

private slots:
	void recompute_disparity();

signals:
	void frame(std::shared_ptr<std::vector<cv::Mat>> images);
	void matcher_updated(matcher_qt_wrapper_base* matcher);
};

} /* namespace stereo_tuner */
} /* namespace reco */

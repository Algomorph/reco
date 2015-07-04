/*
 * hallogplayer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_WORKBENCH_HALLOGPLAYER_H_
#define RECO_WORKBENCH_HALLOGPLAYER_H_
#pragma once

//qt
#include <qobject.h>

//opencv
#include <opencv2/core/core.hpp>

//std
#include <vector>
#include <memory>

//arpg
#include <HAL/Messages/Reader.h>

namespace reco {
namespace workbench {

class hal_log_player: public QObject {
	/**
	 * A player object for playing back HAL log files (implementation deferred/cancelled)
	 */

Q_OBJECT

private:
	bool ready = false;

	bool stop_requested = false;
	std::unique_ptr<std::vector<cv::Mat>> images = std::unique_ptr<std::vector<cv::Mat>>();
	std::unique_ptr<hal::Reader> reader;

public:
	hal_log_player();
	virtual ~hal_log_player();

	void play();
	void stop();
	bool is_ready();

	bool next();
	bool prev();
	void go_to_start();
	void go_to_end();
	void go_to_frame(uint index);
	uint frame_count();



	void open_file(const std::string& path);
	void close_file();



signals:
	void frame(std::vector<cv::Mat>);



};

} /* namespace workbench */
} /* namespace reco */

#endif /* APPLICATIONS_WORKBENCH_HALLOGPLAYER_H_ */

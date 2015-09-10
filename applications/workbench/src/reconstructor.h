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

#include <thread>
#include <mutex>
#include <condition_variable>


namespace reco {
namespace workbench {
/**
 * Responsible for processing the range images and coming up with 3D meshes reconstructed from them
 */
class reconstructor {
private:

	//locking
	std::mutex pause_mutex;
	std::thread thread;
	std::condition_variable pause_cv;

	//thread state variables
	bool paused;
	bool stopped;

	//data processing
	void process_data();
public:


	//thread run management
	void run();
	void pause();
	void stop();

	reconstructor();
	virtual ~reconstructor();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_RECONSTRUCTOR_H_ */

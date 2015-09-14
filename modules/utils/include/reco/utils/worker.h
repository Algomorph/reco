/*
 * worker.h
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_UTILS_WORKER_H_
#define RECO_UTILS_WORKER_H_

//standard
#include <thread>
#include <mutex>
#include <condition_variable>


namespace reco {
namespace utils {
/**
 * Responsible for processing the range images and coming up with 3D meshes reconstructed from them
 */
class worker  {

private:
	//locking
	std::mutex pause_mutex;
	std::thread thread;
	std::condition_variable pause_cv;

	//thread state variables
	bool paused;
	bool stopped;
	//data processing
	void work();

protected:
	/**
	 * Perform a single unit of work
	 * @return true if there is more work to do, false otherwise
	 */
	virtual bool do_unit_of_work() = 0;
	virtual void pre_thread_join();

public:
	//thread run management
	virtual void run();
	void pause();
	virtual void stop();

	worker();
	virtual ~worker();
};

} /* namespace utils */
} /* namespace reco */

#endif /* RECO_UTILS_RECONSTRUCTOR_H_ */

/*
 * reconstructor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/utils/worker.h>
#include <reco/utils/debug_util.h>

namespace reco {
namespace utils {

worker::worker():
	paused(false),
	stopped(false){

}

worker::~worker(){
}

void worker::work(){
	while(!stopped){
		{
		std::unique_lock<std::mutex> lck(pause_mutex);
		//wait if paused
		pause_cv.wait(lck, [&]{return !paused;});
		}
		bool more_work_to_do = true;
		while(!stopped && !paused && more_work_to_do){
			//if no more work to process, flag off
			more_work_to_do = do_unit_of_work();
		}
		stopped = stopped || !more_work_to_do;
	}
}

void worker::run(){
	if(paused){
		std::unique_lock<std::mutex> lck(pause_mutex);
		paused = false;
		pause_cv.notify_one();
	}else{
		this->thread = std::thread(&worker::work, this);
	}
}

void worker::pause(){
	std::unique_lock<std::mutex> lck(pause_mutex);
	paused = true;
}

void worker::pre_thread_join(){

}

void worker::stop(){
	stopped = true;
	if(paused){
		std::unique_lock<std::mutex> lck(pause_mutex);
		paused = false;
		pause_cv.notify_one();
	}
	pre_thread_join();
	if(this->thread.joinable()){
		this->thread.join();
	}
}


} /* namespace utils */
} /* namespace reco */

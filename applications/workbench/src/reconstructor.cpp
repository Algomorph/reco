/*
 * reconstructor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/reconstructor.h>

namespace reco {
namespace workbench {

reconstructor::reconstructor():
	paused(false){

}

reconstructor::~reconstructor(){
	// TODO Auto-generated destructor stub
}

void reconstructor::process_data(){
	while(!stopped){
		{
		std::unique_lock<std::mutex> lck(pause_mutex);
		//wait if paused
		pause_cv.wait(lck, []{return !paused;});
		}
	}
}

void reconstructor::run(){
	if(paused){
		std::unique_lock<std::mutex> lck(pause_mutex);
		paused = false;
		pause_cv.notify_one();
	}else{
		this->thread = std::thread(&reconstructor::process_data, this);
	}
}

void reconstructor::pause(){
	std::unique_lock<std::mutex> lck(pause_mutex);
	paused = true;
}

void reconstructor::stop(){
	paused = false;
	stopped = true;
	this->thread.join();
}


} /* namespace workbench */
} /* namespace reco */

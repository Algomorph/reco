/*
 * thread_runnable.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <reco/datapipe/thread_runnable.h>
//std
#include <thread>

namespace reco {
namespace datapipe {

/**
 *Default constructor
 */
thread_runnable::thread_runnable():run_thread(){
	connect(this, SIGNAL(_stop_requested()), this, SLOT(stop()));
	connect(this, SIGNAL(_pause_requested()), this, SLOT(pause()));
}

thread_runnable::~thread_runnable(){}

void thread_runnable::run_helper(){
	is_paused = false;//reset pause flag in case this was paused before start
	run();
	if(stop_requested){
		emit _stop_requested();
	}else if(pause_requested){

		emit _pause_requested();
	}
}

void thread_runnable::stop(){
	run_thread->join();
	run_thread.reset();
	stop_requested = false;
	emit stopped();
}

void thread_runnable::pause(){
	run_thread->join();
	run_thread.reset();
	pause_requested = false;
	is_paused = true;
	emit paused();
}

/**
 * Start the thread_runnable job until either paused or stopped
 */
void thread_runnable::start(){
	if(!run_thread){
		run_thread.reset(new std::thread(&thread_runnable::run_helper,this));
	}
}



/**
 * Set the stop_requested flag to true
 */
void thread_runnable::request_stop(){
	if(is_paused){
		emit stopped();
	}else{
		stop_requested=true;
	}
}

/**
 * Set the pause_requested flag to true
 */
void thread_runnable::request_pause(){
	if(!is_paused){
		pause_requested=true;
	}
}

} /* namespace datapipe */
} /* namespace reco */

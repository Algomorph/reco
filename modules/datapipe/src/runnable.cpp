/*
 * runnable.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/runnable.h>

namespace reco {
namespace datapipe {

/**
 *
 */
runnable::runnable(): stop_requested(false){}

runnable::~runnable(){}

void runnable::hook_to_thread(QThread* thread){
	connect(thread, SIGNAL(started()), this,SLOT(start()));
	connect(this, SIGNAL(finished()), thread, SLOT(quit()));
	connect(this, SIGNAL(finished()), thread, SLOT(deleteLater()));
	connect(thread, SIGNAL(finished()),thread,SLOT(deleteLater()));
}

/**
 * Reset stop_requested flag and start the runnable job,
 */
void runnable::start(){
	stop_requested = false;
	run();
}

/**
 * Set the stop_requested flag to true
 */
void runnable::request_stop(){
	stop_requested=true;
}

} /* namespace datapipe */
} /* namespace reco */

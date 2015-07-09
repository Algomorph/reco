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
 *Default constructor
 */
runnable::runnable() {
}

runnable::~runnable() {
}

void runnable::run_in_thread() {
	QThread* thread = new QThread;
	this->moveToThread(thread);
	connect(thread, SIGNAL(started()), this, SLOT(start()));
	//on runnable being stopped, quit the thread
	connect(this, SIGNAL(stopped()), thread, SLOT(quit()));
	//mark both object and thread for deletion after stop
	connect(this, SIGNAL(stopped()), thread, SLOT(deleteLater()));
	connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
	thread->start();
}

void runnable::request_start() {
	if (!is_running) {
		is_running = true;
		this->run_in_thread();
	}
}

/**
 * Start the runnable job until either paused or stopped
 */
void runnable::start() {
	is_running = false; //reset pause flag in case this was paused before start

	run();
	if (stop_requested) {
		emit stopped();
		stop_requested = false;
		is_running = false;
	} else if (pause_requested) {
		emit paused();
		is_running = false;
		pause_requested = false;
	}
}

/**
 * Set the stop_requested flag to true
 */
void runnable::request_stop() {
	if (is_running) {
		emit stopped();
	} else {
		stop_requested = true;
	}
}

/**
 * Set the pause_requested flag to true
 */
void runnable::request_pause() {
	if (!is_running) {
		pause_requested = true;
	}
}

} /* namespace datapipe */
} /* namespace reco */

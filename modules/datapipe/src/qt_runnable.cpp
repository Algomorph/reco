/*
 * qt_runnable.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <reco/datapipe/qt_runnable.h>

//std
#include <iostream>

namespace reco {
namespace datapipe {

/**
 *Default constructor
 */
qt_runnable::qt_runnable() :
				stop_requested(false),
				pause_requested(false),
				is_paused(false),
				kinect_data_thread(new QThread) {
	hook_to_thread();
}

qt_runnable::~qt_runnable() {
}

void qt_runnable::hook_to_thread() {

	this->moveToThread(kinect_data_thread);
	//set up error reporting;

	connect(kinect_data_thread, SIGNAL(started()), this, SLOT(start()));
	//on qt_runnable being stopped, quit the thread
	connect(this, SIGNAL(stopped()), kinect_data_thread, SLOT(quit()));
	//mark both object and thread for deletion after stop
	connect(this, SIGNAL(stopped()), kinect_data_thread, SLOT(deleteLater()));
	connect(kinect_data_thread, SIGNAL(finished()), kinect_data_thread, SLOT(deleteLater()));

}

void qt_runnable::request_start() {

	kinect_data_thread->start();
}

/**
 * Start the qt_runnable job until either paused or stopped
 */
void qt_runnable::start() {
	is_paused = false; //reset pause flag in case this was paused before start
	run();
	if (stop_requested) {
		emit stopped();
		stop_requested = false;
	} else if (pause_requested) {
		emit paused();
		is_paused = true;
		pause_requested = false;
	}
}

/**
 * Set the stop_requested flag to true
 */
void qt_runnable::request_stop() {
	if (is_paused) {
		emit stopped();
	} else {
		stop_requested = true;
	}
}

/**
 * Set the pause_requested flag to true
 */
void qt_runnable::request_pause() {
	if (!is_paused) {
		pause_requested = true;
	}
}

} /* namespace datapipe */
} /* namespace reco */

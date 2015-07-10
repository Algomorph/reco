/*
 * THREAD_RUNNABLE.h
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef MODULES_DATAPIPE_SRC_THREAD_RUNNABLE_H_
#define MODULES_DATAPIPE_SRC_THREAD_RUNNABLE_H_

//qt
#include <QObject>
//std
#include <thread>
#include <memory>

namespace reco {
namespace datapipe {
/**
 * Base abstract THREAD_RUNNABLE object that represents a job that can be submitted to a thread
 */
class thread_runnable:
		public QObject {
Q_OBJECT

public:
	thread_runnable();
	virtual ~thread_runnable();

private:
	void run_helper();
	std::shared_ptr<std::thread> run_thread;

protected:
	/**
	 * Primary job function which is executed on the thread hooked to this THREAD_RUNNABLE.
	 * May check the stop_requested flag to safely terminate.
	 */
	virtual void run() = 0;
	/**
	 * Manually set from the caller thread via the request_stop() slot when the job is
	 * requested to stop before finishing.
	 */
	bool stop_requested = false;
	bool pause_requested = false;
	bool is_paused = false;

public slots:
	virtual void start();
	virtual void request_stop();
	virtual void request_pause();

private slots:
	virtual void stop();
	virtual void pause();

signals:
	/**
	 * Emitted on error
	 * @param error
	 */
	void error(QString err);

	void paused();
	void stopped();
	void _stop_requested();
	void _pause_requested();

};

} /* namespace datapipe */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_THREAD_RUNNABLE_H_ */

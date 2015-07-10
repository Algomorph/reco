/*
 * qt_runnable.h
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef MODULES_DATAPIPE_SRC_RUNNABLE_H_
#define MODULES_DATAPIPE_SRC_RUNNABLE_H_

//qt
#include <QObject>
#include <QThread>

namespace reco {
namespace datapipe {
/**
 * Base abstract qt_runnable object that represents a job that can be submitted to a thread
 */
class qt_runnable:
		public QObject {
Q_OBJECT

public:
	qt_runnable();
	virtual ~qt_runnable();


private:
	void hook_to_thread();

protected:
	/**
	 * Primary job function which is executed on the thread hooked to this qt_runnable.
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

	QThread* kinect_data_thread = NULL;

private slots:
	virtual void start();

public slots:
	virtual void request_start();
	virtual void request_stop();
	virtual void request_pause();

signals:
	/**
	 * Emitted on error
	 * @param error
	 */
	void error(QString err);
	void paused();
	void stopped();

};

} /* namespace datapipe */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_RUNNABLE_H_ */

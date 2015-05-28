/*
 * runnable.h
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
 * Base abstract runnable object that represents a job that can be submitted to a thread
 */
class runnable:
		public QObject {
Q_OBJECT

public:
	runnable();
	virtual ~runnable();
	void hook_to_thread(QThread* thread);

protected:
	/**
	 * Primary job function which is executed on the thread hooked to this runnable.
	 * May check the stop_requested flag to safely terminate.
	 */
	virtual void run() = 0;
	/**
	 * Manually set from the caller thread via the request_stop() slot when the job is
	 * requested to stop before finishing.
	 */
	bool stop_requested;

public slots:
	virtual void start();
	virtual void request_stop();

signals:
	/**
	 * Emitted on error
	 * @param error
	 */
	void error(QString err);
	void finished();

};

} /* namespace datapipe */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_RUNNABLE_H_ */

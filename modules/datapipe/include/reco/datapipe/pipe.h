/*
 * pipe.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once


#include <reco/datapipe/typedefs.h>

#include <QObject>

namespace reco {
namespace datapipe {

class pipe:
		public QObject {

Q_OBJECT

public:
	pipe(frame_buffer_type buffer, int num_channels=0);
	virtual ~pipe();

	int get_num_channels();
	frame_buffer_type get_buffer();

protected:
	int num_channels;
	frame_buffer_type buffer;

signals:
	/**
	 * Emitted on error
	 * @param error
	 */
	void error(QString err);
	/**
	 * Emitted when a new frame had been processed and pushed onto the buffer
	 */
	void frame();

};



} /* namespace datapipe */
} /* namespace reco */

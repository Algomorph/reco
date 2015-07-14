/*
 * data_channel.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_DATAPIPE_DATA_CHANNEL_H_
#define RECO_DATAPIPE_DATA_CHANNEL_H_
#pragma once

#include <string>

namespace reco {
namespace datapipe {
/**
 * Contains metainformation about a data channel, such as a video or depth stream.
 */
class data_channel {
private:
	int _height;
	int _width;
	int _offset;
	int _element_size;
	int _chunk_size;
	std::string _name;
public:
	data_channel(std::string name, int height, int width, int offset, int element_size) {
		_height = height;
		_width = width;
		_offset = offset;
		_element_size = element_size;
		_chunk_size = height * width * element_size;
		_name = name;
	}
	~data_channel(){};
	/**
	 * @return height of the data chunk, e.g. height of an image in pixels
	 */
	int height() const {
		return _height;
	}
	/**
	 * @return width of the data chunk, e.g. width of a depth feed in pixels
	 */
	int width() const {
		return _width;
	}
	/**
	 * For instance, a depth camera may produce a feed that has two channels: RGB video and a depth.
	 * Each frame of the feed will include an RGB frame and a depth frame. This defines the relative
	 * offet of the channel within the feed.
	 * @return offset from previous channel in a multi-channel/composite feed.
	 */
	int offset() const {
		return _offset;
	}
	/**
	 * I.e. a 3-channel 8-bit image would have an
	 * element size of 3.
	 * @return bite size of each element within the data chunk
	 */
	int element_size() const {
		return _element_size;
	}

	/**
	 * @return bit size of the whole data chunk
	 */
	int chunk_size() const {
		return _chunk_size;
	}

	std::string name() const {
		return _name;
	}

};

}
}

#endif /* RECO_DATAPIPE_DATA_CHANNEL_H_ */

/*
 * swap_buffer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_UTILS_SWAP_BUFFER_H_
#define RECO_UTILS_SWAP_BUFFER_H_
#pragma once

namespace reco {
namespace utils {

/**
 * Abstract 1-slot queue
 **/
template<typename T> class swap_buffer {

public:
	virtual void push_back(const T& item) = 0;
	virtual	T pop_front() = 0;
protected:
	swap_buffer(){};
	virtual ~swap_buffer(){};


};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_UTILS_SWAP_BUFFER_H_ */

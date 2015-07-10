/*
 * swap_buffer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_UTILS_OPTIMISTIC_SWAP_BUFFER_H_
#define RECO_UTILS_OPTIMISTIC_SWAP_BUFFER_H_
#pragma once


//local
#include <reco/utils/swap_buffer.h>

//std
#include <vector>
#include <atomic>

namespace reco {
namespace utils {


template<typename T>
class optimistic_swap_buffer: public swap_buffer<T> {
	/**
	 * A thread-safe 1-slot queue with optimistic waiting (requires non-0 items)
	 **/
public:
	optimistic_swap_buffer();
	virtual ~optimistic_swap_buffer();
	virtual void push_back(const T& item);
	virtual T pop_front();

private:
	std::atomic_bool allow_pop;
	std::atomic_bool allow_push;
	std::atomic<T> storage_a;
	T storage[1];
};

template<typename T> optimistic_swap_buffer<T>::optimistic_swap_buffer() :
				allow_pop(false),
				allow_push(true) {
}

template<typename T> optimistic_swap_buffer<T>::~optimistic_swap_buffer() {

}

template<typename T> void optimistic_swap_buffer<T>::push_back(const T& item) {
	bool exp;
	//wait for item to get popped
	do {
		exp = true;
	} while (!allow_push.compare_exchange_weak(exp, false)); //cork up the bottle as soon as we get through
	memcpy(&storage[0], &item, sizeof(T));
	allow_pop.store(true);
}

template<typename T> T optimistic_swap_buffer<T>::pop_front() {
	bool exp;
	//wait for item to get pushed
	do {
		exp = true;
	} while (!allow_pop.compare_exchange_weak(exp, false)); //cork up the bottle as soon as we get through
	T ret;
	memcpy(&ret, &storage[0], sizeof(T));
	allow_push.store(true);
	return ret;
}

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_UTILS_OPTIMISTIC_SWAP_BUFFER_H_ */

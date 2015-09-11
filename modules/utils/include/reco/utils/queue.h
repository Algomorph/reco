/*
 * queue.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_UTILS_QUEUE_H_
#define RECO_UTILS_QUEUE_H_
#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace reco {
namespace utils {

template<typename T>
class queue{

private:
	std::queue<T> internal_queue;
	std::mutex mutex;
	std::condition_variable cond;

public:
	T pop(){
		std::unique_lock<std::mutex> mlock(mutex);
		while (internal_queue.empty()){
			cond.wait(mlock);
		}
		auto item = internal_queue.front();
		internal_queue.pop();
		return item;
	}

	void pop(T& item){
		std::unique_lock<std::mutex> mlock(mutex);
		while (internal_queue.empty()){
			cond.wait(mlock);
		}
		item = internal_queue.front();
		internal_queue.pop();
	}

	void push(const T& item){
		std::unique_lock<std::mutex> mlock(mutex);
		internal_queue.push(item);
		mlock.unlock();
		cond.notify_one();
	}

	void push(T&& item){
		std::unique_lock<std::mutex> mlock(mutex);
		internal_queue.push(std::move(item));
		mlock.unlock();
		cond.notify_one();
	}

};

} //end namespace utils
} //end namespace reco

#endif /* RECO_UTILS_QUEUE_H_ */

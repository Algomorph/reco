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

/**
 * Abstract  queue
 **/
template<typename T>
class queue {

public:
	virtual void push_back(const T& item) = 0;
	virtual T pop_front() = 0;
	virtual void clear() = 0;

	protected:
	queue() {
	}
	;
	virtual ~queue() {
	}
	;

};

/**
 * A thread-safe queue using pessimistic locking
 */

template<typename T>
class unbounded_queue{

private:
	std::queue<T> internal_queue;
	std::mutex mutex;
	std::condition_variable cond;

public:
	T pop_front(){
		std::unique_lock<std::mutex> mlock(mutex);
		while (internal_queue.empty()){
			cond.wait(mlock);
		}
		auto item = internal_queue.front();
		internal_queue.pop();
		return item;
	}

	void pop_front(T& item){
		std::unique_lock<std::mutex> mlock(mutex);
		while (internal_queue.empty()){
			cond.wait(mlock);
		}
		item = internal_queue.front();
		internal_queue.pop();
	}

	void push_back(const T& item){
		std::unique_lock<std::mutex> mlock(mutex);
		internal_queue.push(item);
		mlock.unlock();
		cond.notify_one();
	}

	void push_back(T&& item){
		std::unique_lock<std::mutex> mlock(mutex);
		internal_queue.push(std::move(item));
		mlock.unlock();
		cond.notify_one();
	}

	void clear(){
		std::unique_lock<std::mutex> mlock(mutex);
		while(!internal_queue.empty()){
			internal_queue.pop();
		}
		mlock.unlock();
	}

	size_t size(){
		return internal_queue.size();
	}

};

} //end namespace utils
} //end namespace reco

#endif /* RECO_UTILS_QUEUE_H_ */

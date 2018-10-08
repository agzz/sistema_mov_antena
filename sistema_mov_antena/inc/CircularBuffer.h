/*
 * CircularBuffer.h
 *
 * 2018 Adriano Zenzen
 *
 * Based on code from:
 * https://github.com/embeddedartistry/embedded-resources
 *
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <cstdio>
#include <memory>

/**
 * Important Usage Note: This library reserves one spare entry for queue-full detection
 * Otherwise, corner cases and detecting difference between full/empty is hard.
 * You are not seeing an accidental off-by-one.
 */

template <class T>
class CircularBuffer {
public:
	CircularBuffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		size_(size)
	{
		//Empty constructor
	}

	void put(T item) {
		buf_[head_] = item;
		head_ = (head_ + 1) % size_;

		if (head_ == tail_) {
			tail_ = (tail_ + 1) % size_;
		}
	}

	T get(void) {
		if (empty()) {
			return T();
		}

		//Read data and advance the tail (we now have a free space)
		auto val = buf_[tail_];
		tail_ = (tail_ + 1) % size_;

		return val;
	}

	T peek(void) {
		if (empty()) {
			return T();
		}

		//Read data
		auto val = buf_[tail_];

		return val;
	}

	void reset(void) {
		head_ = tail_;
	}

	bool empty(void) {
		//if head and tail are equal, we are empty
		return head_ == tail_;
	}

	bool full(void) {
		//If tail is ahead the head by 1, we are full
		return ((head_ + 1) % size_) == tail_;
	}

	size_t size(void) {
		return ((head_-tail_)&(size_ - 1));
	}

	size_t capacity(void) {
		return size_ - 1;
	}

private:
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	size_t size_;
};

#endif

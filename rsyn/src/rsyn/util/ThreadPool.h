/* Copyright 2014-2018 Rsyn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#ifndef UTIL_THREAD_POOL_H
#define UTIL_THREAD_POOL_H

////////////////////////////////////////////////////////////////////////////////
// Guilherme Flach: 2012-02-24
//
// This code was an adaptation from
// https://github.com/progschj/ThreadPool/
//
// The main change I did was to make to task not required to return a 
// std::future and added a method to wait for all threads to finish. Note that 
// with std::future one did not need a wait method. But this was weird when the
// task should not return anything (i.e. void).
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Example
////////////////////////////////////////////////////////////////////////////////
//
//	ThreadPool pool;
//	for (int i = 0; i < 10; ++i) {
//		pool.addTask([i] {
//			std::cout << "hello " << i << std::endl;
//			std::this_thread::sleep_for(std::chrono::seconds(1));
//			std::cout << "world " << i << std::endl;
//		});
//	} // end for
//
//	pool.wait(); // wait for all tasks to be finished
//
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

class ThreadPool {
public:

	// Start the thread pool with the number of available threads in the
	// computer (using std::thread::hardware_concurrency()). If it fails to
	// query the number of threads, the thread pool is initialized with 2 
	// threads
	ThreadPool();

	// Start the thread pool with the specified number of threads.
	ThreadPool(const std::size_t numThreads);

	// Add a task.
	template<class F, class... Args>
	void addTask(F&& f, Args&&... args);

	// Wait for all threads to finish.
	void wait();
	
	// Get the number of working threads.
	std::size_t getNumThreads() const { return workers.size(); }
	
	// Stop all threads.
	~ThreadPool();

private:
	// need to keep track of threads so we can join them
	std::vector<std::thread> workers;
	// the task queue
	std::queue<std::function<void()>> tasks;

	// synchronization
	std::mutex queue_mutex;
	std::condition_variable condition;
	std::condition_variable condition_wait_task;
	bool stop;
	std::atomic<int> running;

	// Manage workers.
	void startWorkers(const std::size_t numThreads);
	void stopWorkers();
}; // end class

// -----------------------------------------------------------------------------

inline ThreadPool::ThreadPool(const std::size_t numThreads) {
	startWorkers(numThreads);
} // end method

// -----------------------------------------------------------------------------

inline ThreadPool::ThreadPool() {
	startWorkers(std::max(2u, std::thread::hardware_concurrency()));
} // end method

// -----------------------------------------------------------------------------

inline void ThreadPool::startWorkers(const std::size_t numThreads) {
	stop = false;
	running = 0;

	for (std::size_t i = 0; i < numThreads; i++) {
		workers.emplace_back([this] {
			while (true) {
				std::function<void() > task;

				{ // mutual exclusion block
					std::unique_lock<std::mutex> lock(queue_mutex);
					condition.wait(lock, [this] {return stop || !tasks.empty();});
					if (stop && tasks.empty())
						return;
					task = std::move(tasks.front());
					tasks.pop();
				} // end block

				running++;
				task();
				running--;
				condition_wait_task.notify_one();
			} // end while
		});
	} // end method	
} // end method

// -----------------------------------------------------------------------------

inline void ThreadPool::stopWorkers() {
	{ // mutual exclusion block
		std::unique_lock<std::mutex> lock(queue_mutex);
		stop = true;
	} // end block

	condition.notify_all();
	for (std::thread &worker : workers) {
		worker.join();
	} // end for
} // end method

// -----------------------------------------------------------------------------

template<class F, class... Args>
void ThreadPool::addTask(F&& f, Args&&... args) {
	auto task = std::make_shared<std::packaged_task<void()>>(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...));

	{ // mutual exclusion block
		std::unique_lock<std::mutex> lock(queue_mutex);

		// don't allow enqueueing after stopping the pool
		if (stop) {
			throw std::runtime_error("Adding a task on a stopped ThreadPool.");
		} // end if

		tasks.emplace([task](){(*task)();});
	} // end block
	condition.notify_one();
} // end method

// -----------------------------------------------------------------------------

inline void ThreadPool::wait() {
	std::unique_lock<std::mutex> lock(queue_mutex);
	condition_wait_task.wait(lock, [this] {return running == 0;});		
} // end method

// -----------------------------------------------------------------------------

inline ThreadPool::~ThreadPool() {
	stopWorkers();
} // end destructor

#endif
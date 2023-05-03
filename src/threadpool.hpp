//
// Created by Sohonet on 2/16/23.
//

#ifndef ULTRAGRID_THREADPOOL_H
#define ULTRAGRID_THREADPOOL_H

#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <thread>


template <typename T>
class ThreadPool {
public:
    explicit ThreadPool(std::vector<T> threadResources, int threadCount = -1);
    ThreadPool(const ThreadPool& threadpool);
    void Start();
    void QueueJob(const std::function<void(T)>& job);
    void Stop();
    bool Busy();
    static unsigned int ThreadCount();

    std::vector<T> threadResources;
private:
    void workerLoop(int threadIndex);

    bool shouldTerminate = false;           // Tells threads to stop looking for jobs
    std::mutex queueMutex;                  // Prevents data races to the job queue
    std::condition_variable mutexCondition; // Allows threads to wait on new jobs or termination
    std::vector<std::thread> threads;
    std::queue<std::function<void(T)>> jobs;
    int threadCount;
};

/**
 * @brief An implementation of the default constructor which initialises all of the members.
*/
template <typename T>
ThreadPool<T>::ThreadPool(std::vector<T> threadResources, int threadCount) : threadResources(threadResources), shouldTerminate(false), queueMutex(),
                                                                             mutexCondition(), threads(), jobs(), threadCount(threadCount) {
    // If no specific thread count has been specified then use the hardware concurrency value
    if(this->threadCount == -1) {
        this->threadCount = std::thread::hardware_concurrency();
    }
    // Assert that there are enough thread resources being made for the threads.
    assert(this->threadResources.size() >= this->threadCount);
}

/**
 * @brief An implementation of the copy constructor. This will only copy the contents of the
 *        shouldTerminate flag and simply initialise the mutex, condition, threads and jobs.
*/
template <typename T>
ThreadPool<T>::ThreadPool(const ThreadPool<T>& threadpool) : queueMutex(), mutexCondition(), threads(), jobs() {
    this->threadCount = threadpool.threadCount;
    this->threadResources = threadpool.threadResources;
    this->shouldTerminate = threadpool.shouldTerminate;
}

/**
 * @brief In order to have an efficient thread pool architecture we will run a set
 *        amount of threads, running worker loops that will execute 'jobs' from a queue.
 */
template <typename T>
void ThreadPool<T>::Start() {
    // Set up a number of thread workers to perform the given tasks.
    this->threads.resize(this->threadCount);

    // Create the thread workers
    for(uint32_t i = 0; i < this->threadCount; i++) {
        threads.at(i) = std::thread(&ThreadPool::workerLoop, this, i);
    }
}

/**
 * @brief An infinite loop worker thread that will acquire jobs from the job queue and execute them.
 */
template <typename T>
void ThreadPool<T>::workerLoop(int threadIndex) {
    while(true) {
        // Set up a job variable to collect the job off of the queue
        std::function<void(T)> job;
        // Start a new lifecycle context so ensure the lock is released after we have acquired a job from
        // the queue.
        {
            // Try getting the lock used to access members of the job queue
            std::unique_lock<std::mutex> lock = std::unique_lock<std::mutex>(this->queueMutex);

            // Sit and wait on the mutex and exit only if there is a job to execute or if the thread pool has been
            // asked to terminate.
            this->mutexCondition.wait(lock, [this] {
                return !this->jobs.empty() || this->shouldTerminate;
            });

            // If termination has been requested, then we should break out of the loop
            // so the thread running this stops.
            if(this->shouldTerminate) {
                break;
            }

            // Otherwise, we can only leave the mutex condition if there is a job waiting for us, and we
            // have acquired the lock. Create a reference to the object at the front of the queue, and then
            // pop it off of the queue so another
            job = this->jobs.front();
            this->jobs.pop();
        }
        // Leave the context, which will release & destroy the lock
        // Run the collected job.
        job(this->threadResources.at(threadIndex));
    }
}

/**
 * @brief A function for enqueueing a job into the thread pool
 */
template <typename T>
void ThreadPool<T>::QueueJob(const std::function<void(T)>& job) {
    // Create a lifecycle context so that the lock is released and destroyed at the end of it.
    {
        // Acquire the lock for modifying the job queue and push the job into the queue
        std::unique_lock<std::mutex> lock = std::unique_lock<std::mutex>(this->queueMutex);
        this->jobs.push(job);
    }
    // Notify all worker threads that a job has been added to the queue and that the lock is free.
    this->mutexCondition.notify_one();
}

/**
 * @brief A function for checking if the thread pool is currently busy (is it waiting to process
 *        additional jobs).
 *
 *        WARNING - This function does not indicate that the threadpool has finished processing
 *                  all jobs, simply that there are no more jobs in the queue for it to begin
 *                  processing. Calling the "stop" function may take time to fully execute as
 *                  there may still be threads executing.
 */
template <typename T>
bool ThreadPool<T>::Busy() {
    bool poolBusy;
    // Create a lifecycle context so that the lock is released and destroyed at the end of it
    {
        std::unique_lock<std::mutex> lock = std::unique_lock<std::mutex>(this->queueMutex);
        poolBusy = this->jobs.empty();
    }
    return !poolBusy;
}

/**
 * @brief A function for shutting down the active list of threads and joining them. This will mark that the
 *        worker threads should be terminated, and then notify all waiting threads to wake up and check the
 *        termination conditions so that they will exit.
 */
template <typename T>
void ThreadPool<T>::Stop() {
    // Create a lifecycle context so that the lock is released and destroyed at the end of it
    {
        // Grab the lock and update the internal loop variable to be true (stopping the inifinite loops).
        std::unique_lock<std::mutex> lock = std::unique_lock<std::mutex>(this->queueMutex);
        this->shouldTerminate = true;
    }
    // Notify all waiting threads that there has been an update (causing them to drop out on the next) conditional
    // check.
    this->mutexCondition.notify_all();

    // Join all threads and destroy all references to them
    for(std::thread& activeThread : this->threads) {
        activeThread.join();
    }
    this->threads.clear();
}

/**
 * @brief A helper function for being able to query the amount of threads
 *        that would be used by the pool.
 *
 * @return - unsigned int - The amount of threads that are supported concurrently
 *                          by the running machine.
 */
template <typename T>
unsigned int ThreadPool<T>::ThreadCount() {
    return std::thread::hardware_concurrency();
}

#endif //ULTRAGRID_THREADPOOL_H

//
// Created by carlostojal on 08-01-2024.
//

#include <local_mapping_ros/post_processing/ThreadPool.h>

namespace t24e::local_mapper {

    ThreadPool::ThreadPool(size_t numWorkers=0) {

        this->numWorkers = numWorkers;

        this->currThreadIdx = 0;

    }

    ThreadPool::~ThreadPool() {
        // kill all before finishing
        this->killAll();
    }

    void ThreadPool::start() {

        this->shouldStop = false;

        // if no number was specified, use the number of threads of the CPU
        if(this->numWorkers <= 0)
            this->numWorkers = std::thread::hardware_concurrency();

        // put all workers on the loop
        for(size_t i = 0; i < this->numWorkers; i++) {
            this->workers.emplace_back(std::thread(&ThreadPool::threadLoop, this));
        }

        // notify all workers to start
        this->queueConditionVariable.notify_all();
    }

    void ThreadPool::threadLoop() {

        // run indifenitely
        while(true) {
            
            std::function<void(size_t threadIdx)> job;
            size_t threadIdx;

            {
                // acquire the mutex
                std::unique_lock<std::mutex> lk(this->queueMutex);

                // wait for the condition variable
                this->queueConditionVariable.wait(lk, [this]{
                    return !this->jobQueue.empty() || this->shouldStop;
                });
                if(this->shouldStop)
                    return;

                // increment the thread index
                threadIdx = this->currThreadIdx++;
                
                // select the next job to be done
                job = this->jobQueue.front();
                // remove the job from the queue
                this->jobQueue.pop();

                {
                    // acquire the completion mutex
                    std::unique_lock<std::mutex> lk(this->completionMutex);

                    // increment completed jobs count
                    this->numCompletedJobs++;

                    // if all the jobs were complete, notify the completion condition variable
                    if(this->numCompletedJobs == this->numEnqueuedJobs)
                        this->completionConditionVariable.notify_all();
                }
            }

            // start the job, receiving its thread index
            job(threadIdx);

            // notify the next free worker
            this->queueConditionVariable.notify_one();
        }

    }

    void ThreadPool::queueJob(std::function<void(size_t threadIdx)> job) {

        {
            // acquire the job queue mutex
            std::unique_lock<std::mutex> lk(this->queueMutex);

            // add the job to the queue
            this->jobQueue.push(job);

            {
                // acquire the completion mutex
                std::unique_lock<std::mutex> lk(this->completionMutex);

                // increment the enqueued jobs count
                this->numEnqueuedJobs++;
            }
        }

        // notify the next free worker
        this->queueConditionVariable.notify_one();
    }

    void ThreadPool::killAll() {
    
        {
            // acquire the queue mutex
            std::unique_lock<std::mutex> lk(this->queueMutex);

            // mark to stop
            this->shouldStop = true;
        }

        // notify all threads to stop runnning the loop
        this->queueConditionVariable.notify_all();

        // wait for each worker
        for(std::thread& t : this->workers) {
            t.join();
        }

        // clear the vector
        this->workers.clear();
    }

    bool ThreadPool::isBusy() {

        bool isBusy;
        {
            // acquire the mutex
            std::unique_lock<std::mutex> lk(this->queueMutex);

            // the thread is not busy if the pool has jobs enqueued
            isBusy = !this->jobQueue.empty();
        }
        return isBusy;
    }

    void ThreadPool::join() {

        // acquire the completion mutex
        std::unique_lock<std::mutex> lk(this->completionMutex);

        // wait for the completion condition variable
        this->completionConditionVariable.wait(lk, [this]{
            return this->numCompletedJobs == this->numEnqueuedJobs;
        });
    }

    size_t ThreadPool::getNumWorkers() const {
        return this->numWorkers;
    }
}
//
// Created by carlostojal on 08-01-2024.
//

#include <local_mapping_ros/post_processing/ThreadPool.h>

namespace t24e::local_mapper {

    ThreadPool::ThreadPool(size_t numWorkers=0) {

        // if no number was specified, use the number of threads of the CPU
        if(numWorkers <= 0)
            this->numWorkers = std::thread::hardware_concurrency();
        else
            this->numWorkers = numWorkers;

        // put all workers on the loop
        for(size_t i = 0; i < this->numWorkers; i++) {
            this->workers.emplace_back(std::thread(&ThreadPool::threadLoop, this));
        }

    }

    void ThreadPool::threadLoop() {

        // run indifenitely
        while(true) {
            
            std::function<void()> job;
            {
                // acquire the mutex
                std::unique_lock<std::mutex> lk(this->queueMutex);

                // wait for the condition variable
                this->queueConditionVariable.wait(lk, [this]{
                    return !this->jobQueue.empty() || this->shouldStop;
                });
                if(this->shouldStop)
                    return;
                
                // select the next job to be done
                job = this->jobQueue.front();
                // remove the job from the queue
                this->jobQueue.pop();
            }
            // start the job
            job();
        }

    }

    void ThreadPool::queueJob(std::function<void(void)> job) {

        {
            // acquire the job queue mutex
            std::unique_lock<std::mutex> lk(this->queueMutex);

            // add the job to the queue
            this->jobQueue.push(job);
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

    size_t ThreadPool::getNumWorkers() const {
        return this->numWorkers;
    }
}
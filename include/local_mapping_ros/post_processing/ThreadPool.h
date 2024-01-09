//
// Created by carlostojal on 08-01-2024.
//

#ifndef LOCAL_MAPPING_CORE_THREADPOOL_H
#define LOCAL_MAPPING_CORE_THREADPOOL_H

#include <vector>
#include <queue>
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>

namespace t24e::local_mapper {

    /*! \brief Thread pool of fixed size for work scheduling. */
    class ThreadPool {

        private:
            /*! \brief Number of workers. */
            size_t numWorkers;

            /*! \brief Vector of workers. */
            std::vector<std::thread> workers;

            /*! \brief Queue of pending work. */
            std::queue<std::function<void(size_t threadIdx)>> jobQueue;

            /*! \brief Queue condition variable. Controls scheduling. */
            std::condition_variable queueConditionVariable;

            /*! \brief Queue mutex. Prevents race conditions on the queue. */
            std::mutex queueMutex;

            /*! \brief Should the pool stop? */
            bool shouldStop = false;

            /*! \brief Incremental index of the current thread. To be used by the programmer has he wishes. */
            size_t currThreadIdx = 0;

            /*! \brief Total count of enqueued jobs. */
            size_t numEnqueuedJobs = 0;

            /*! \brief Total count of finished jobs. */
            size_t numCompletedJobs = 0;

            std::mutex completionMutex;

            /*! \brief Completion condition variable. Notified every time a job finishes. */
            std::condition_variable completionConditionVariable;

            /*! \brief Loop run by each thread in the pool waiting for work. */
            void threadLoop();


        public:
            /*! \brief Initialize a pool with a number of threads. */
            ThreadPool(size_t numWorkers);

            /*! \brief Start the thread pool. */
            void start();

            /*! \brief Schedule a job to the pool queue. */
            void queueJob(std::function<void(size_t threadIdx)> job);

            /*! \brief Notify all workers to stop its job asap. */
            void killAll();

            /*! \brief Is the thread busy at the moment? */
            bool isBusy();

            /*! \brief Wait for the pool to finish the currently enqueued work. */
            void join();

            size_t getNumWorkers() const;

    };
}

#endif // LOCAL_MAPPING_CORE_THREADPOOL_H
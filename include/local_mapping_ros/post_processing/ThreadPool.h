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
            std::queue<std::function<void(void)>> jobQueue;

            /*! \brief Queue condition variable. Controls scheduling. */
            std::condition_variable queueConditionVariable;

            /*! \brief Queue mutex. Prevents race conditions on the queue. */
            std::mutex queueMutex;


        public:
            /*! \brief Initialize a pool with a number of threads. */
            ThreadPool(size_t numWorkers);

            /*! \brief Schedule a job to the pool queue. */
            void queueJob(std::function<void(void)> job);

    };
}

#endif // LOCAL_MAPPING_CORE_THREADPOOL_H
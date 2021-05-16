#pragma once

#include "ThreadSafeQueue.hpp"
#include "ScopedThread.hpp"

#include <vector>
#include <memory>
#include <thread>
#include <future>
#include <functional>
#include <stdexcept>
#include <atomic>

namespace crsGA
{

class ThreadPool
{
  private:
    void runPendingTask();

  public:
    ThreadPool(size_t numThreads = std::thread::hardware_concurrency());
    template <class F, class... Args>
    auto enqueue(F &&f, Args &&... args) -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();

    size_t getNumThreads() const { return workers.size(); }

    static ThreadPool &instance()
    {
        static ThreadPool s_pool;
        return s_pool;
    }

  private:
    // stop guard
    std::atomic<bool> stop;
    // the task queue
    ThreadSafeQueue<std::function<void()>> tasks;
    // need to keep track of threads so we can join them
    std::vector<ScopedThread> workers;
};

// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads) : stop(false),
                                                tasks(),
                                                workers(threads)
{
    for (size_t i = 0; i < threads; ++i)
        workers[i] = std::thread(
            [this] {
                while (!stop)
                {
                    runPendingTask();
                }
            });
}

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::enqueue(F &&f, Args &&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    typedef typename std::result_of<F(Args...)>::type return_type;

    // don't allow enqueueing after stopping the pool
    if (stop)
        throw std::runtime_error("enqueue on stopped ThreadPool");

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    tasks.push([task]() { (*task)(); });
    return res;
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    stop = true;
    tasks.notify_and_terminate();
    workers.clear();
}

inline void ThreadPool::runPendingTask()
{
    std::function<void()> task;
    if (this->tasks.wait_pop(task))
    {
        task();
    }
}

} // namespace crsGA

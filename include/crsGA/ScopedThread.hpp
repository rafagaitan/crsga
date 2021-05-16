#pragma once

#include <thread>
#include <stdexcept>
#include <functional>

namespace crsGA
{

class ScopedThread
{
    std::thread _t;

  public:
    explicit ScopedThread() : _t()
    {
    }
    ScopedThread(std::thread t) : _t(std::move(t))
    {
        if (!_t.joinable())
            throw std::logic_error("Thread already joined");
    }
    explicit ScopedThread(ScopedThread &&st) : _t(std::move(st._t))
    {
    }

    template <class F, class... Args>
    explicit ScopedThread(F &&f, Args &&... args) : _t(std::bind(std::forward<F>(f), std::forward<Args>(args)...))
    {
    }

    ScopedThread &operator=(ScopedThread &&st)
    {
        _t = std::move(st._t);
        return *this;
    }
    ScopedThread &operator=(std::thread &&t)
    {
        _t = std::move(t);
        return *this;
    }
    ~ScopedThread()
    {
        if (_t.joinable())
            _t.join();
    }
    ScopedThread(const ScopedThread &rhs) = delete; // deleted
  private:
    ScopedThread &operator=(const ScopedThread &rhs); // deleted
};

} // namespace crsGA

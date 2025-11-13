#pragma once

#include "AsyncOptimizer.hpp"

namespace mesh_mppi
{

template<typename KinematicT>
AsyncOptimizer<KinematicT>::AsyncOptimizer()
: stop_thread_(false)
{
    thread_ = std::thread(std::bind(&AsyncOptimizer::threadRun, this));
}

template<typename KinematicT>
AsyncOptimizer<KinematicT>::Future AsyncOptimizer<KinematicT>::computeOptimalControl(const State& initial)
{
    Future fut;
    {
        std::lock_guard guard(mutex_);
        input_ = initial;
        promise_ = Promise();
        fut = promise_.get_future();
    }
    // Start the thread
    notify_.notify_all();

    return fut;
}

template<typename KinematicT>
void AsyncOptimizer<KinematicT>::threadRun()
{
    while (!stop_thread_)
    {
        std::unique_lock lock(mutex_);
        notify_.wait(lock, [&](){ return input_.has_value() || stop_thread_;});
        auto t0 = std::chrono::steady_clock::now();

        if (stop_thread_)
        {
            break;
        }

        State input = input_.value();
        Promise prom = std::move(promise_);
        input_.reset();
        lock.unlock();
        
        auto result = OptimizerT::computeOptimalControl(input);
        
        prom.set_value(std::move(result));
        auto t1 = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(OptimizerT::get_logger(), "Async Optimization Time: %li", duration_cast<milliseconds>(t1 - t0).count());
    }
}

template <typename KinematicT>
AsyncOptimizer<KinematicT>::~AsyncOptimizer()
{
    stop_thread_.store(true);
    notify_.notify_all();
    if (thread_.joinable())
    {
        thread_.join();
    }
}

} // mesh_mppi

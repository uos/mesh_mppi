#pragma once

#include "MPPIOptimizer.hpp"

namespace mesh_mppi
{

template <typename KinematicT>
class AsyncOptimizer: public MPPIOptimizer<KinematicT>
{
    using OptimizerT = MPPIOptimizer<KinematicT>;
public:
    // Result of the optimization process
    using Result = typename std::expected<typename OptimizerT::Result, Error>;
    using Promise = typename std::promise<Result>;
    using Future = typename std::future<Result>;

    AsyncOptimizer();

    // Initialization is not thread safe
    using OptimizerT::initialize;
    using OptimizerT::setDistributionSigma;
    using OptimizerT::setNumSamples;
    using OptimizerT::setPredictionHorizon;
    using OptimizerT::numControlSignals;
    using OptimizerT::reset;


    /**
     *  @brief Start async computation
     *
     *  @param initial The state to compute the optimal control for
     *  @return A future to retrieve the result once its done
     */
    Future computeOptimalControl(const State& initial);

    virtual ~AsyncOptimizer();

private:

    using OptimizerT::computeOptimalControl;

    void threadRun();

private:
    // Synchronization
    std::mutex mutex_;

    // Notify for new inputs
    std::condition_variable notify_;

    // Input state
    std::optional<State> input_;

    // Output promise
    Promise promise_;

    // Thread
    std::thread thread_;

    // Thread stop
    std::atomic_bool stop_thread_;
};


} // namespace mesh_mppi

#include "AsyncOptimizer.inl"

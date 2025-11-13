#pragma once

#include <cstddef>

namespace mesh_mppi
{

/**
 *  @brief Base interface for Optimizer implementations.
 *
 *  Allows to access the number of control signals and to set the prediction horizon
 *  the number of samples and the sampling distribution sigmas.
 */
class OptimizerBase
{
public:

    /**
    *   @brief Set the number of samples to use in the optimization
    */
    virtual void setNumSamples(size_t n) = 0;

    /**
    *   @brief Set the number of elements per sampled trajectory
    */
    virtual void setPredictionHorizon(size_t n) = 0;

    /**
    *   @brief Set the standard deviation of the noise sampling distribution for control signal i.
    */
    virtual void setDistributionSigma(const size_t i, const float sigma) = 0;

    /**
     *  @brief Get the number of control signals used by this optimizer (Equal to Kinematic control signals)
     */
    virtual size_t numControlSignals() const = 0;

    /**
     *  @brief Reset the internal state of the optimizer. Called when a new path is set and the robot is not moving
     */
    virtual void reset() = 0;

};

} // namespace mesh_mppi

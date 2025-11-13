#pragma once
#include "mesh_mppi/kinematics/KinematicsBase.hpp"
#include "mesh_mppi/types/State.hpp"
#include "mesh_mppi/types/Pose.hpp"

#include <geometry_msgs/msg/twist.hpp>

namespace mesh_mppi
{

/**
 *  @brief Templated Kinematics to provide the interface parts which are dependent on the number of control and state variables.
 */
template <uint32_t N_CONTROL, uint32_t N_STATE>
class Kinematics: public KinematicsBase
{
public:
    static constexpr const uint32_t ControlSignals = N_CONTROL;
    static constexpr const uint32_t StateVariables = N_STATE;

    using SharedPtr = typename std::shared_ptr<Kinematics>;
    using ControlVector = typename Eigen::Vector<float, N_CONTROL>;
    using StateVector = typename Eigen::Vector<float, N_STATE>;
    
    /**
     * @brief Returns the number of control signals used by the plugin.
     *
     * For example the DifferentialDrive uses 2 control signals,
     * linear velocity and angular velocity.
     * The interpretation of the control signal is up to the Kinematics
     */
    [[nodiscard]]
    static constexpr uint32_t numControlSignals() noexcept
    {
        return N_CONTROL;
    }
    
    /**
     * @brief Returns the number of state variables used by the plugin.
     *
     * For example the diffdrive has 2 state variables, linear and angular vel
     */
    [[nodiscard]]
    static constexpr uint32_t numStateVariables() noexcept
    {
        return N_STATE;
    }

    /**
     *  @brief Deduce the control signals from the current state "inverse kinematic"
     */
    [[nodiscard]]
    virtual StateVector getInitialStateFromVelocity(const geometry_msgs::msg::Twist& vel) const noexcept = 0;

    /**
     *  @brief Get the State X_{t+1} from X_t and u_t
     */
    [[nodiscard]]
    virtual StateVector getNextState(const StateVector& current, const ControlVector& u) const = 0;

    /**
    * @brief Calculate the planar Pose delta from a given velocity
    */
    [[nodiscard]]
    virtual Pose2D getPoseDelta(const StateVector& state) const noexcept = 0;
    
    /**
    *   Apply limits to the control signals based on the current state.
    *   e.g.: Acceleration Limits to not overshoot allowed velocities
    */
    virtual void applyConstraints(const StateVector& current, ControlVector& action) const noexcept = 0;

    /**
    *   Generate a Twist msg from a control signal.
    *   TODO:: Replace this by publishing custom msg
    */
    virtual geometry_msgs::msg::Twist getTwistFromControls(
        const State& state,
        const ControlVector& controls
    ) const noexcept = 0;
};

} // namespace mesh_mppi

#pragma once

#include "Kinematics.hpp"
#include <rclcpp/node.hpp>

namespace mesh_mppi::kinematics
{

class DifferentialDriveKinematics
: public Kinematics<2, 2>
{
public:
    using SharedPtr = typename std::shared_ptr<DifferentialDriveKinematics>;
    
    /// Initialize the kinematics
    bool initialize() override final;
    
    /// Translate velocity to state
    StateVector getInitialStateFromVelocity(const geometry_msgs::msg::Twist& vel) const noexcept override final;
    
    /// State transition
    StateVector getNextState(const StateVector& current, const ControlVector& u) const noexcept override final;

    /// Get the 2D pose delta resulting from the velocity
    Pose2D getPoseDelta(const StateVector& state) const noexcept override final;
    
    /// Apply kinematic constraints to the control signals
    void applyConstraints(const StateVector& current, ControlVector& action) const noexcept override final;

    /// Generate twist from controls
    geometry_msgs::msg::Twist getTwistFromControls(
        const State& state,
        const ControlVector& controls
    ) const noexcept override final;

private:
    
    // ROS Parameter handling
    rcl_interfaces::msg::SetParametersResult reconfigure_callback(
        std::vector<rclcpp::Parameter> parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    // Velocity limits
    struct {
        float linear_vel_min;
        float linear_vel_max;
        float angular_vel_max;
        float linear_acc_min;
        float linear_acc_max;
        float angular_acc_max;
    } limits_;
};


}

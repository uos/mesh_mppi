#pragma once

#include "Kinematics.hpp"

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace mesh_mppi::kinematics
{

class BicycleKinematics
: public Kinematics<2, 3>
{
public:
    using SharedPtr = typename std::shared_ptr<BicycleKinematics>;
    
    /// Initialize the kinematic
    bool initialize() override final;
    
    /// Translate velocity to state
    StateVector getInitialStateFromVelocity(const geometry_msgs::msg::Twist& vel) const noexcept override final;
    
    /// State transition
    StateVector getNextState(const StateVector& current, const ControlVector& u) const noexcept override final;

    /// Get the 2D pose delta resulting from the velocity
    Pose2D getPoseDelta(const StateVector& state) const noexcept override final;
    
    /// Apply kinematic constraints to the control signals
    void applyConstraints(const StateVector& state, ControlVector& action) const noexcept override final;

    /// Generate twist from controls
    geometry_msgs::msg::Twist getTwistFromControls(
        const State& state,
        const ControlVector& controls
    ) const noexcept override final;

private:

    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& states);

    // ROS Parameter handling
    rcl_interfaces::msg::SetParametersResult reconfigure_callback(
        std::vector<rclcpp::Parameter> parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    // Limits limits
    struct {
        // Linear velocity min
        float linear_vel_min;
        // Linear velocity max
        float linear_vel_max;
        // Linear acceleration max
        float linear_acc_max;
        // Linear acceleration min
        float linear_acc_min;
        // Maxmimum Steering Angle
        float steer_max;
        // Steering velocity gain
        float steer_vel_gain;
        // Steering joint velocity max
        float steer_vel_max;
        // Distance between the read and front axle
        float wheelbase;
        // Distance from the robot base link to the rear axle
        float rear_dist;
    } limits_;

    // Stores a joint state
    struct JointState
    {
        std::string name = "";
        float position = 0.0;
        float velocity = 0.0;
        float effort = 0.0;
    };

    struct {
        JointState left_joint;
        JointState right_joint;
    } joints_;
    // The mutex is mutable to allow synchronization in const methods
    mutable std::mutex joint_mtx_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::CallbackGroup::SharedPtr cbk_group_;
    
    // Ackermann Drive pub
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;
};


}

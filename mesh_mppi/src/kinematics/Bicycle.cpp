#include <mesh_mppi/kinematics/Bicycle.hpp>
#include <mesh_mppi/types/Velocity.hpp>
#include <cmath>

namespace mesh_mppi::kinematics
{

using StateVector = BicycleKinematics::StateVector;
using ControlVector = BicycleKinematics::ControlVector;

bool BicycleKinematics::initialize()
{
    using rcl_interfaces::msg::ParameterDescriptor;
    using rcl_interfaces::msg::FloatingPointRange;
    std::map<std::string, std::pair<double, ParameterDescriptor>> parameters;

    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_velocity_max";
        descriptor.description = "The maximum linear velocity";
        parameters.insert(
            {descriptor.name, {0.5, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_velocity_min";
        descriptor.description = "The minimum linear velocity";
        parameters.insert(
            {descriptor.name, {0.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_acceleration_max";
        descriptor.description = "The maximum linear acceleration";
        parameters.insert(
            {descriptor.name, {3.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_acceleration_min";
        descriptor.description = "The minimum linear acceleration";
        parameters.insert(
            {descriptor.name, {3.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "steering_angle_max";
        descriptor.description = "The maximum steering angle";
        parameters.insert(
            {descriptor.name, {1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "steering_joint_velocity_gain";
        descriptor.description = "The gain for the steering joint velocity. V_{t+1} = V_{t} + d_{angle} * gain * dt";
        parameters.insert(
            {descriptor.name, {1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "steering_joint_velocity_max";
        descriptor.description = "The maximum velocity the steering joint can reach";
        parameters.insert(
            {descriptor.name, {1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "wheelbase";
        descriptor.description = "The distance between the front and rear axle";
        parameters.insert(
            {descriptor.name, {1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "rear_axle_distance";
        descriptor.description = "The distance between the robot base link and rear axle";
        parameters.insert(
            {descriptor.name, {0.0, descriptor}}
        );
    }

    dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
        &BicycleKinematics::reconfigure_callback, this, std::placeholders::_1
    ));

    node_->declare_parameters(name_, parameters);

    {
        ParameterDescriptor descriptor;
        descriptor.name = name_ + ".left_steer_joint";
        descriptor.description = "The left joint used for steering";

        joints_.left_joint.name = node_->declare_parameter<std::string>(descriptor.name, descriptor);
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = name_ + ".right_steer_joint";
        descriptor.description = "The right joint used for steering";

        joints_.right_joint.name = node_->declare_parameter<std::string>(descriptor.name, descriptor);
    }

    // Use a callback group to allow multithreaded execution
    cbk_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbk_group_;
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable().durability_volatile(),
        std::bind(&BicycleKinematics::joint_state_callback, this, std::placeholders::_1),
        options
    );

    ackermann_pub_ = node_->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ackermann_drive", rclcpp::QoS(10).reliable().durability_volatile());

    return true;
}


rcl_interfaces::msg::SetParametersResult BicycleKinematics::reconfigure_callback(
    std::vector<rclcpp::Parameter> parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;


    for (const auto& param: parameters)
    {
        if (name_ + ".linear_velocity_max" == param.get_name())
        {
            limits_.linear_vel_max = param.as_double();
        }
        else if (name_ + ".linear_velocity_min" == param.get_name())
        {
            limits_.linear_vel_min = param.as_double();
        }
        else if (name_ + ".linear_acceleration_max" == param.get_name())
        {
            limits_.linear_acc_max = param.as_double();
        }
        else if (name_ + ".linear_acceleration_min" == param.get_name())
        {
            limits_.linear_acc_min = param.as_double();
        }
        else if (name_ + ".steering_angle_max" == param.get_name())
        {
            limits_.steer_max = param.as_double();
        }
        else if (name_ + ".steering_joint_velocity_gain" == param.get_name())
        {
            limits_.steer_vel_gain = param.as_double();
        }
        else if (name_ + ".steering_joint_velocity_max" == param.get_name())
        {
            limits_.steer_vel_max = param.as_double();
        }
        else if (name_ + ".wheelbase" == param.get_name())
        {
            limits_.wheelbase = param.as_double();
        }
        else if (name_ + ".rear_axle_distance" == param.get_name())
        {
            limits_.rear_dist = param.as_double();
        }
    }

    return result;
}


StateVector BicycleKinematics::getInitialStateFromVelocity(
    const geometry_msgs::msg::Twist& vel
) const noexcept
{
    std::lock_guard lock(joint_mtx_);
    const float avg_position = (joints_.left_joint.position + joints_.right_joint.position) * 0.5;
    const float avg_vel = (joints_.left_joint.velocity + joints_.right_joint.velocity) * 0.5;

    // The state is linear velocity, steering angle and steering velocity
    return StateVector(vel.linear.x, avg_position, avg_vel);
}


StateVector BicycleKinematics::getNextState(
    const StateVector& current,
    const ControlVector& u
) const noexcept
{
    const float dt = getDeltaTime();
    StateVector res = StateVector::Zero();
    res(0) = u(0);
    
    // U(1) is the desired steering angle
    const float delta = u(1) - current(1);

    // New Joint velocity, model a pid gain like the Gazebo AckermannSteering
    res(2) = delta * limits_.steer_vel_gain;

    // Limit joint velocity
    res(2) = std::clamp(res(2), -limits_.steer_vel_max, limits_.steer_vel_max);

    // New Joint position
    res(1) = current(1) + res(2) * dt; // Apply steering velocity
    res(1) = std::clamp(res(1), -limits_.steer_max, limits_.steer_max);

    return res;
}


Pose2D BicycleKinematics::getPoseDelta(const StateVector& state) const noexcept
{
    Pose2D res;

    // Odometry formula from "Mobile Roboter" by Joachim Hertzberg et. al.
    // Calculate twist at a reference point with distance lh to the rear axle

    /// Distance from the point of interest (robot base frame) to the rear axle
    const float beta = std::atan(limits_.rear_dist * std::tan(state(1)) / limits_.wheelbase);
    const float v = (state(0) * std::cos(state(1)) + state(0)) / (2.0 * std::cos(beta));
    const float theta = ((v * std::cos(beta)) / limits_.wheelbase) * std::tan(state(1));


    // Position delta. Having dt twice is correct because we want the proper angle delta and the proper linear vel delta
    const float dt = getDeltaTime();
    res.x = v * std::cos(beta + theta * 0.5 * dt) * dt;
    res.y = v * std::sin(beta + theta * 0.5 * dt) * dt;

    // Orientation delta
    res.theta = theta * dt;

    return res;
}


void BicycleKinematics::applyConstraints(
    const StateVector& state,
    ControlVector& action
) const noexcept
{
    // Determine max and min linear acceleration from current state
    {
        const float upper = state(0) + limits_.linear_acc_max * getDeltaTime();
        const float lower = state(0) + limits_.linear_acc_min * getDeltaTime();
        action(0) = std::clamp(action(0), lower, upper);
        action(0) = std::clamp(action(0), limits_.linear_vel_min, limits_.linear_vel_max);
    }
    {
        // Limit steering angle
        action(1) = std::clamp(action(1), -limits_.steer_max, limits_.steer_max);
    }
}


geometry_msgs::msg::Twist BicycleKinematics::getTwistFromControls(
    const State& state,
    const ControlVector& controls
) const noexcept
{
    (void) state;
    Velocity vel;
    // Command linear velocity is the same
    vel.linear.x = controls(0);
    // Inverse of the decoding in getInitialStateFromVelocity
    vel.angular.z = controls(0) * (std::tan(controls(1)) / limits_.wheelbase);

    // TODO: This needs to be part of the kinematic interface
    ackermann_msgs::msg::AckermannDrive msg;
    msg.speed = controls(0);
    msg.steering_angle = controls(1);
    ackermann_pub_->publish(msg);

    return Velocity::toMsg(vel);
}

void BicycleKinematics::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& states)
{
    std::lock_guard lock(joint_mtx_);
    // Check all provided states
    for (size_t i = 0; i < states->name.size(); i++)
    {
        JointState* joint = nullptr;
        if (joints_.left_joint.name == states->name[i])
        {
            joint = &joints_.left_joint;
        }
        else if (joints_.right_joint.name == states->name[i])
        {
            joint = &joints_.right_joint;
        }
        else
        {
            continue;
        }

        if (states->position.empty() || states->velocity.empty())
        {
            RCLCPP_ERROR_ONCE(node_->get_logger(), "JointState for joint '%s' has no position or velocity information!", joint->name.c_str());
            return;
        }

        // Copy the state
        joint->position = states->position[i];
        joint->velocity = states->velocity[i];

        if (!states->effort.empty())
        {
            joint->effort = states->effort[i];
        }
    }
}
} // namespace mesh_mppi

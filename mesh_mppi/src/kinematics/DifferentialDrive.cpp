#include <mesh_mppi/kinematics/DifferentialDrive.hpp>

namespace mesh_mppi::kinematics
{

using StateVector = DifferentialDriveKinematics::StateVector;
using ControlVector = DifferentialDriveKinematics::ControlVector;

bool DifferentialDriveKinematics::initialize()
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
        descriptor.name = "angular_velocity_max";
        descriptor.description = "The maximum angular velocity";
        parameters.insert(
            {descriptor.name, {M_PI, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_acceleration_max";
        descriptor.description = "The maximum linear acceleration";
        parameters.insert(
            {descriptor.name, {0.5, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "linear_acceleration_min";
        descriptor.description = "The minimum linear acceleration";
        parameters.insert(
            {descriptor.name, {-1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "angular_acceleration_max";
        descriptor.description = "The maximum angular acceleration";
        parameters.insert(
            {descriptor.name, {M_PI, descriptor}}
        );
    }

    dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
        &DifferentialDriveKinematics::reconfigure_callback, this, std::placeholders::_1
    ));

    node_->declare_parameters(name_, parameters);

    return true;
}


rcl_interfaces::msg::SetParametersResult DifferentialDriveKinematics::reconfigure_callback(
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
        else if (name_ + ".angular_velocity_max" == param.get_name())
        {
            limits_.angular_vel_max = param.as_double();
        }
        else if (name_ + ".linear_acceleration_max" == param.get_name())
        {
            limits_.linear_acc_max = param.as_double();
        }
        else if (name_ + ".linear_acceleration_min" == param.get_name())
        {
            limits_.linear_acc_min = param.as_double();
        }
        else if (name_ + ".angular_acceleration_max" == param.get_name())
        {
            limits_.angular_acc_max = param.as_double();
        }
    }

    return result;
}


StateVector DifferentialDriveKinematics::getInitialStateFromVelocity(
    const geometry_msgs::msg::Twist& vel
) const noexcept
{
    return StateVector(vel.linear.x, vel.angular.z);
}


StateVector DifferentialDriveKinematics::getNextState(
    const StateVector& current,
    const ControlVector& u
) const noexcept
{
    // Maybe more complex acceleration smoothed next state?
    (void) current;
    return u;
}


Pose2D DifferentialDriveKinematics::getPoseDelta(const StateVector& state) const noexcept
{
    Pose2D res;

    const float& v = state(0);
    const float& theta = state(1);
    const float& dt = getDeltaTime();

    // Position delta
    res.x = v * std::cos(theta * 0.5 * dt) * dt;
    res.y = v * std::sin(theta * 0.5 * dt) * dt;

    // Orientation delta
    res.theta = theta * dt;

    return res;
}


void DifferentialDriveKinematics::applyConstraints(
    const StateVector& state,
    ControlVector& action
) const noexcept
{
    // Apply limits for max and min linear velocity and enforce max acceleration limits
    {
        const float& max_acc = limits_.linear_acc_max;
        const float& min_acc = limits_.linear_acc_min;
        const float upper = state(0) + max_acc * getDeltaTime();
        const float lower = state(0) + min_acc * getDeltaTime();
        action(0) = std::clamp(action(0), lower, upper);
        action(0) = std::clamp(action(0), limits_.linear_vel_min, limits_.linear_vel_max);
    }
    {
        const float& max_acc = limits_.angular_acc_max;
        const float upper = state(1) + max_acc * getDeltaTime();
        const float lower = state(1) - max_acc * getDeltaTime();
        action(1) = std::clamp(action(1), lower, upper);
        action(1) = std::clamp(action(1), -limits_.angular_vel_max, limits_.angular_vel_max);
    }
}


geometry_msgs::msg::Twist DifferentialDriveKinematics::getTwistFromControls(
    const State& state,
    const ControlVector& controls
) const noexcept
{
    (void) state;
    geometry_msgs::msg::Twist msg;
    msg.linear.x = controls(0);
    msg.angular.z = controls(1);
    return msg;
}

} // namespace mesh_mppi

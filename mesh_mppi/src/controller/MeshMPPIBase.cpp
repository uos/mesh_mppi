#include <cmath>
#include <format>

#include <mesh_mppi/controller/MeshMPPIBase.hpp>
#include <mesh_mppi/Util.hpp>

#include <mbf_msgs/action/exe_path.hpp>

#include <mesh_map/util.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>


namespace mesh_mppi
{

using Eigen::Vector3f;
using ExePathResult = mbf_msgs::action::ExePath::Result;

void MeshMPPIBase::setCurrentPoseAndVelocity(
    const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Twist& twist
)
{
    pose_.position.x() = pose.position.x;
    pose_.position.y() = pose.position.y;
    pose_.position.z() = pose.position.z;
    pose_.orientation.x() = pose.orientation.x;
    pose_.orientation.y() = pose.orientation.y;
    pose_.orientation.z() = pose.orientation.z;
    pose_.orientation.w() = pose.orientation.w;
    pose_.velocity.fromMsg(twist);
    pose_.valid = true;
}

lvr2::OptionalFaceHandle MeshMPPIBase::getCurrentFace() const
{
    return pose_.faceH;
}


bool MeshMPPIBase::updateCurrentFace(const State& current)
{
    auto vec = current.pose.position;
    if (!pose_.faceH)
    {
        // TODO: This should be a parameter
        auto opt = map_->getContainingFace(vec, 0.3);
        if (opt)
        {
            // Make sure pos is on the surface
            pose_.faceH = opt;
        }
        else
        {
            return false;
        }
    }
    else // We have an initial guess
    {
        std::array<float, 3> bary;
        float dist = 0.0;
        // Check if the position is still inside the face
        if (mesh_map::projectedBarycentricCoords(
            vec,
            map_->mesh()->getVertexPositionsOfFace(pose_.faceH.unwrap()),
            bary,
            dist
        ) && dist < 0.3)
        {
            // Face stays the same
            return true;
        }
        // Check the neighbouring faces
        else if (auto search_opt = map_->searchNeighbourFaces(
            vec, pose_.faceH.unwrap(), 0.3, 0.3))
        {
            // We found the neighbour face
            pose_.faceH = std::get<0>(search_opt.value());
        }
        else if (auto search_opt = map_->searchContainingFace(
            vec, 0.3))
        {
            pose_.faceH = std::get<0>(search_opt.value());
        }
        else
        {
            return false;
        }
    }

    return true;
}


bool MeshMPPIBase::initializeParameters()
{
    // Configure the parameters
    { // The prediction horizon of the controller
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The number of steps in the prediction";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 10;
    range.to_value = 100;
    descriptor.integer_range.push_back(range);
    params_.horizon = node_->declare_parameter(name_ + ".optimizer.prediction_steps", 30, descriptor);
    }
    { // The number of samples in the sampling
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The number of samples in the prediction";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 100000;
    descriptor.integer_range.push_back(range);
    params_.samples = node_->declare_parameter(name_ + ".optimizer.prediction_samples", 1000, descriptor);
    }
    { // The distribution sigmas sampled in the optimizer
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = std::format("The standard deviations used to sample the {} control signal noises", getOptimizerBase().numControlSignals());
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = -10.0;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    const std::vector<double> sigmas = node_->declare_parameter<std::vector<double>>(
        std::format("{}.optimizer.stddev", name_), descriptor
    );
    for (size_t i = 0; i < getOptimizerBase().numControlSignals(); i++)
    {
        getOptimizerBase().setDistributionSigma(i, sigmas[i]);
    }
    }
    { // The map cost limit used for collision checks
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = name_ + ".map_cost_limit";
    desc.description = "The limit at/above which map_costs are considered lethal. Should match the inscribed radius cost of the InflationLayer."
            "This parameter is used for all collision checks.";
    params_.map_cost_limit = node_->declare_parameter<double>(desc.name, 1.0, desc);
    }
    // read move_base_flex's controller frequency
    if (!node_->has_parameter("controller_frequency"))
    {
        // Default frequency in move_base_flex::AbstractControllerExecution is 20Hz
        node_->declare_parameter<double>("controller_frequency", 20.0);
    }
    params_.controller_frequency = node_->get_parameter("controller_frequency").as_double();

    { // Translation threshold for progress check
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = name_ + ".progress_check.translation_threshold";
    desc.description = "The minimum distance the robot has to travel in the specified time frame to not be considered stuck.";
    progress_translation_threshold_ = node_->declare_parameter<double>(desc.name, 0.25, desc);
    }
    { // Translation timeframe for progress check
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = name_ + ".progress_check.timeframe";
    desc.description = "The timeframe in seconds to use for the controllers's progress check.";
    const double time = node_->declare_parameter<double>(desc.name, 5.0, desc);
    if (0.0 >= time)
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid parameter value: '%s' cannot be less or equal to 0.0!", desc.name.c_str());
        return false;
    }
    progress_num_timesteps_ = time * params_.controller_frequency;
    }

    reconfigure_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&MeshMPPIBase::reconfigure, this, std::placeholders::_1)
    );
    
    return true;
}


rcl_interfaces::msg::SetParametersResult MeshMPPIBase::reconfigure(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param: parameters)
    {
        // Update the kinematic models delta time if the controller_frequency changes
        if ("controller_frequency" == param.get_name())
        {
            params_.controller_frequency = param.as_double();
            getKinematicsBase().setDeltaTime(1.0 / param.as_double());
        }
        else if (name_ + ".progress_check.translation_threshold" == param.get_name())
        {
            progress_translation_threshold_ = param.as_double();
        }
        else if (name_ + ".progress_check.timeframe" == param.get_name())
        {
            if (0.0 >= param.as_double())
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Invalid parameter value: '%s.progress_check.timeframe' cannot be less or equal to 0.0!",
                    name_.c_str()
                );
                result.reason = "Invalid parameter value: 'progress_check.timeframe' cannot be less or equal to 0.0!";
                result.successful = false;
            }
            progress_num_timesteps_ = params_.controller_frequency * param.as_double();
        }
    }

    return result;
}


bool MeshMPPIBase::isGoalReached(double dist_tolerance, double angle_tolerance)
{
    if (plan_.empty())
    {
        return true;
    }
    
    // When a new plan is set we might be called before we
    // were able to update the pose. This is relevant for simulation
    // experiments and if the robot is moved via teleop between
    // executions. In theory only when the goal is close to the last
    // stored pose.
    if (!pose_.valid)
    {
        return false;
    }

    Eigen::Vector3f goal(
        plan_.back().pose.position.x,
        plan_.back().pose.position.y,
        plan_.back().pose.position.z
    );
    Quaternion orientation(
        plan_.back().pose.orientation.w,
        plan_.back().pose.orientation.x,
        plan_.back().pose.orientation.y,
        plan_.back().pose.orientation.z
    );
    reached_goal_ = (goal - pose_.position).norm() < dist_tolerance && orientation.angularDistance(pose_.orientation) < angle_tolerance;
    return reached_goal_;
}


bool MeshMPPIBase::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
    if (reached_goal_)
    {
        getOptimizerBase().reset();
        first_ = true;
    }

    plan_ = plan;
    reached_goal_ = false;
    pose_.valid = false;
    past_trajectory_.clear();
    cost_function_->set_plan(plan);
    return true;
}


void MeshMPPIBase::publishOptimalTrajectory(const Trajectory& trajectory)
{
    // Publish the optimal trajectory
    // Create path msg
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_->mapFrame();
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_->mapFrame();
        for (const State& state: trajectory)
        {
            pose.pose.position.x = state.pose.position.x;
            pose.pose.position.y = state.pose.position.y;
            pose.pose.position.z = state.pose.position.z;
            pose.pose.orientation.x = state.pose.orientation.x();
            pose.pose.orientation.y = state.pose.orientation.y();
            pose.pose.orientation.z = state.pose.orientation.z();
            pose.pose.orientation.w = state.pose.orientation.w();
            msg.poses.push_back(pose);
        }
    }

    traj_pub_->publish(msg);
}

void MeshMPPIBase::publishOptimalControlSequence(const Eigen::ArrayXf& data, uint32_t timesteps, uint32_t signals, const rclcpp::Time& timestamp)
{
    if (sequence_pub_->get_subscription_count() == 0)
    {
        return;
    }

    auto msg = std::make_unique<ControlSequenceStamped>();
    msg->header.stamp = timestamp;

    msg->sequence.num_timesteps = timesteps;
    msg->sequence.num_signals = signals;
    msg->sequence.data = std::vector<float>(data.begin(), data.end());

    sequence_pub_->publish(std::move(msg));
}


bool MeshMPPIBase::initialize(
    const std::string& name,
    const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
    const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
    const rclcpp::Node::SharedPtr& node
)
{
    // Init the controller
    name_ = name;
    tf_ = tf_ptr;
    map_ = mesh_map_ptr;
    node_ = node;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliable();
    qos.keep_last(1);
    traj_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/optimal_trajectory", qos);
    sequence_pub_ = node_->create_publisher<ControlSequenceStamped>("~/optimal_sequence", qos);

    if (!this->initializeParameters())
    {
        RCLCPP_ERROR(node_->get_logger(), "MeshMPPI: Could not initialize parameters!");
        return false;
    }
        
    cost_function_ = std::make_shared<CostFunction>(node_, name, map_);
    if (!cost_function_)
    {
        RCLCPP_ERROR(node->get_logger(), "MeshMPPI: make_unique<CostFunction> returned nullptr");
        return false;
    }

    return this->initialize();
}


bool MeshMPPIBase::isMakingProgress(const Trajectory& traj)
{
    past_trajectory_.push_back(traj.front().pose.position);
    while (past_trajectory_.size() > progress_num_timesteps_)
    {
        past_trajectory_.pop_front();
    }

    // We give the robot time to start moving
    if (past_trajectory_.size() < progress_num_timesteps_)
    {
        return true;
    }

    float traveled_distance = 0.0f;
    for (size_t i = 0, ii = 1; ii < past_trajectory_.size(); i++, ii++)
    {
        traveled_distance += past_trajectory_[i].distance(past_trajectory_[ii]);
    }

    // TODO: Should we also have a rotational term to allow for in place rotation?
    // TODO: Check if we are close to the goal

    return traveled_distance > progress_translation_threshold_;
}

} // namespace mesh_mppi

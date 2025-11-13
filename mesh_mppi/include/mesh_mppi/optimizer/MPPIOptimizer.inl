#pragma once
#include "MPPIOptimizer.hpp"
#include <mesh_mppi/types/Pose.hpp>
#include <random>
#include <chrono>

namespace mesh_mppi
{

using std::chrono::milliseconds;

template <typename KinematicT>
bool MPPIOptimizer<KinematicT>::initialize(
    const rclcpp::Node::SharedPtr& node,
    const std::string& name,
    const std::shared_ptr<CostFunction>& costfunction,
    const std::shared_ptr<KinematicT>& kinematic,
    const mesh_map::MeshMap::Ptr& map,
    const int32_t samples,
    const int32_t horizon
)
{
    this->node_ = node;
    this->cost_function_ = costfunction;
    this->kinematic_ = kinematic;
    this->map_ = map;
    this->params_.samples = 0;
    this->params_.horizon = 0;
    this->setNumSamples(samples);
    this->setPredictionHorizon(horizon);
    this->prev_control_.setZero();
    this->control_history_.setZero();
    this->sequence_.setZero();
    this->namespace_ = std::format("{}.optimizer", name);

    model_ = std::make_unique<SurfaceModel>(map);
    if (!model_)
    {
        RCLCPP_ERROR(get_logger(), "MeshMPPI: make_unique<Model> returned nullptr");
        return false;
    }
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliable();
    qos.keep_last(1);
    this->pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/samples", qos);
    this->cost_pub_ = node->create_publisher<std_msgs::msg::Float32>("~/cost", qos);

    // Parameters
    callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&MPPIOptimizer<KinematicT>::reconfigureCallback, this, std::placeholders::_1));

    params_.temperature = node->declare_parameter(std::format("{}.temperature", namespace_), 4.0);
    params_.gamma = node->declare_parameter(std::format("{}.gamma", namespace_), 0.015);
    
    return true;
}


template <typename KinematicT>
rcl_interfaces::msg::SetParametersResult MPPIOptimizer<KinematicT>::reconfigureCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard lock(mtx_);

    for (const auto& param: parameters)
    {
        if (namespace_ + ".temperature" == param.get_name())
        {
            params_.temperature = param.get_value<double>();
        }
        else if (namespace_ + ".gamma" == param.get_name())
        {
            params_.gamma = param.get_value<double>();
        }
    }

    result.successful = true;
    return result;
}


template <typename KinematicT>
void MPPIOptimizer<KinematicT>::publishTrajectorySamples(
    const std::vector<Trajectory>& trajectories,
    const std::vector<bool>& lethal
)
{
    if (0 == pub_->get_subscription_count())
    {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = map_->mapFrame();
    msg.header.stamp = this->node_->get_clock()->now();

    msg.ns = "mpc_samples";
    msg.id = 0;
    msg.frame_locked = true;

    msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0.05;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.scale.x = 0.015;
    msg.scale.y = 0.015;
    msg.scale.z = 0.015;
    msg.color.a = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;

    visualization_msgs::msg::Marker lethals = msg;
    lethals.ns = "mpc_lethals";
    lethals.color.b = 0.0;
    lethals.color.r = 1.0;
    
    for (size_t i = 0; i < trajectories.size(); i++)
    {
        const auto& traj = trajectories[i];

        if (lethal[i])
        {
            for (const auto& state: traj)
            {
                geometry_msgs::msg::Point pt;
                pt.x = state.pose.position.x;
                pt.y = state.pose.position.y;
                pt.z = state.pose.position.z;
                lethals.points.push_back(pt);
            }
        }
        else
        {
            for (const auto& state: traj)
            {
                geometry_msgs::msg::Point pt;
                pt.x = state.pose.position.x;
                pt.y = state.pose.position.y;
                pt.z = state.pose.position.z;
                msg.points.push_back(pt);
            }
        }
    }

    pub_->publish(msg);
    pub_->publish(lethals);
}

template <typename KinematicT>
void MPPIOptimizer<KinematicT>::updateOptimalControlSequence(const Eigen::ArrayXXf& U, const Eigen::ArrayXf& weights)
{
    const float norm = 1.0f / weights.sum();
    // Average by exponential map of the score
    sequence_.setZero();
    for (Eigen::Index t = 0; t < params_.horizon; t++)
    {
        for (Eigen::Index s = 0; s < KinematicT::numControlSignals(); s++)
        {
            for (Eigen::Index r = 0; r < params_.samples; r++)
            {
                const Eigen::Index idx = t * KinematicT::numControlSignals() + s;
                sequence_(idx) += weights(r) * norm * U.row(r)(idx);
            }
        }
    }

    sequence_ = smoother_.smooth(control_history_, sequence_);
}

template <typename KinematicT>
std::expected<typename MPPIOptimizer<KinematicT>::Result, Error> MPPIOptimizer<KinematicT>::computeOptimalControl(const State& initial)
{
    std::lock_guard lock(mtx_);
    
    // Sample new trajectories
    auto t0 = std::chrono::steady_clock::now();

    for (auto row: samples_.rowwise())
    {
        // Copy the sample
        row = sequence_.transpose();
        
        // Add noise
        for (int t = 0; t < params_.horizon; t++)
        {
            for (uint32_t s = 0; s < KinematicT::numControlSignals(); s++)
            {
                row(t * KinematicT::numControlSignals() + s) += params_.distribtutions[s](rand_);
            }
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(get_logger(), "Sampling: %li", duration_cast<milliseconds>(t1 - t0).count());
    
    t0 = std::chrono::steady_clock::now();

    std::vector<bool> lethal(params_.samples, false);

    // Rollout all trajectories
    t0 = std::chrono::steady_clock::now();
    this->rolloutTrajectories(initial, trajectories_, lethal);
    t1 = std::chrono::steady_clock::now();
    std::chrono::nanoseconds prediction_t = t1 - t0;
    RCLCPP_DEBUG(get_logger(), "Prediction: %li", duration_cast<milliseconds>(prediction_t).count());

    // Prepare the cost function with the initial state
    t0 = std::chrono::steady_clock::now();
    cost_function_->prepare_for_scoring(initial, params_.horizon, trajectories_);

    // Score all trajectories
    for (int i = 0; i < params_.samples; i++)
    {
        // Skip already invalid trajectories
        if (lethal[i])
        {
            scores_(i) = std::numeric_limits<float>::infinity();
            continue;
        }
        // Score the trajectory using the cost function
        if (const auto result = cost_function_->score(samples_.row(i), trajectories_[i]))
        {
            scores_(i) = result.value();
        }
        else
        {
            lethal[i] = true;
            scores_(i) = std::numeric_limits<float>::infinity();
        }
    }

    // Check if a valid command exists
    if (std::count(lethal.begin(), lethal.end(), false) == 0)
    {
        samples_.setZero();
        scores_.setOnes();
        return std::unexpected(Error::NO_VALID_COMMAND);
    }
    // =================== Control Cost ==============================
    const auto N_CONTROL = KinematicT::numControlSignals();
    for (Eigen::Index sample = 0; sample < params_.samples; sample++)
    {
        const auto& control = samples_.row(sample);
        for (Eigen::Index t = 0; t < params_.horizon; t++)
        {
            for (Eigen::Index sig = 0; sig < N_CONTROL; sig++)
            {
                const float u_star = sequence_(t * N_CONTROL + sig);
                const float u = control(t * N_CONTROL + sig);
                scores_(sample) += params_.gamma * (u_star * (u - u_star)) / (params_.distribtutions[sig].stddev() * params_.distribtutions[sig].stddev());
            }
        }
    }

    float min_valid_cost = std::numeric_limits<float>::infinity();
    float max_valid_cost = 0.0;
    for (Eigen::Index i = 0; i < scores_.rows(); i++)
    {
        if (lethal[i])
        {
            continue;
        }
        min_valid_cost = std::min(min_valid_cost, scores_(i));
        max_valid_cost = std::max(max_valid_cost, scores_(i));
    }

    t1 = std::chrono::steady_clock::now();
    std::chrono::nanoseconds scoring_t = t1 - t0;

    RCLCPP_DEBUG(get_logger(), "Scoring: %li", duration_cast<milliseconds>(scoring_t).count());
    t0 = std::chrono::steady_clock::now();

    // =================== MPPI Optimization =========================
    
    RCLCPP_DEBUG(get_logger(), "Valid Cost Range: [%f - %f]", min_valid_cost, max_valid_cost);

    const float baseline = scores_.minCoeff();
    const Eigen::ArrayXf exp_map = ((scores_ - baseline) * -params_.temperature).exp();
    RCLCPP_DEBUG(get_logger(), "Baseline Cost: %f", baseline);

    this->updateOptimalControlSequence(samples_, exp_map);

    State kstate = initial;
    Trajectory final_traj;
    final_traj.reserve(params_.horizon);
    for (auto control: sequence_.reshaped<Eigen::RowMajor>(params_.horizon, KinematicT::numControlSignals()).rowwise())
    {
        ControlVector c(control.transpose());
        kinematic_->applyConstraints(kstate.state, c);
        kstate = this->predictNextState(kstate, c).value_or(kstate);
        final_traj.push_back(kstate);
        control = c.transpose();
    }

    // Check that the final trajectory is valid and publish the cost
    auto result = cost_function_->score(sequence_, final_traj);
    if (!result)
    {
        return std::unexpected(Error::NO_VALID_COMMAND);
    }

    // ===============================================================

    t1 = std::chrono::steady_clock::now();

    this->publishTrajectorySamples(trajectories_, lethal);
    this->cost_pub_->publish(std_msgs::msg::Float32().set__data(result.value()));
    
    
    // TODO: Should the control history be store by the MeshMPPIBase class?
    // The optimizer could be called multiple times and then the history is wrong
    const ControlVector best_control = sequence_.topRows<KinematicT::numControlSignals()>();
    prev_control_ = best_control;
    control_history_.template topRows<3>() = control_history_.template bottomRows<3>();
    control_history_.template bottomRows<1>() = best_control;

    // Shift by 1 control for the next iteration
    this->shiftControlSequence();

    return Result{best_control, sequence_, final_traj};
}


template <typename KinematicT>
void MPPIOptimizer<KinematicT>::rolloutTrajectories(const State& initial, std::vector<Trajectory>& traj_out, std::vector<bool>& invalid_out)
{
    traj_out.resize(params_.samples);
    for (auto& traj: traj_out)
    {
        traj.reserve(params_.horizon);
    }
    invalid_out.resize(params_.samples);

    for (int i = 0; i < params_.samples; i++)
    {
        auto& traj = traj_out[i];
        traj.clear();
        const State* current = &initial;
        // Score s_i,t
        for (int t = 0; t < params_.horizon; t++)
        {
            ControlVector u = samples_.row(i).middleCols<KinematicT::numControlSignals()>(t * KinematicT::numControlSignals());
            kinematic_->applyConstraints(current->state, u);
            samples_.row(i).middleCols<KinematicT::numControlSignals()>(t * KinematicT::numControlSignals()) = u;

            if (auto res = this->predictNextState(*current, u))
            {
                traj.push_back(res.value());
                current = &traj.back();
            }
            else
            {
                invalid_out[i] = true;
                break;
            }
        }
    }
}


template <typename KinematicT>
void MPPIOptimizer<KinematicT>::shiftControlSequence()
{
    constexpr const auto nsigs = KinematicT::numControlSignals();
    sequence_.head((params_.horizon - 1) * nsigs) = sequence_.tail((params_.horizon - 1) * nsigs);
    sequence_.tail(nsigs) = sequence_.segment<nsigs>((params_.horizon - 2) * nsigs);
}


template <typename KinematicT>
void MPPIOptimizer<KinematicT>::setNumSamples(size_t n)
{
    std::lock_guard lock(mtx_);
    const int delta = n - params_.samples;
    samples_.conservativeResize(n, Eigen::NoChange);
    scores_.conservativeResize(n);
    params_.samples = n;
    
    // New values are uninitialized even with conservativeResize
    if (delta > 0)
    {
        samples_.bottomRows(delta).setZero();
        scores_.bottomRows(delta).setZero();
    }
}


template <typename KinematicT>
void MPPIOptimizer<KinematicT>::setPredictionHorizon(size_t n)
{
    std::lock_guard lock(mtx_);
    const int delta = n - params_.horizon;
    samples_.conservativeResize(Eigen::NoChange, n * KinematicT::numControlSignals());
    sequence_.conservativeResize(n * KinematicT::numControlSignals());
    params_.horizon = n;

    // New values are uninitialized even with conservativeResize
    if (delta > 0)
    {
        samples_.rightCols(delta * KinematicT::numControlSignals()).setZero();
        sequence_.bottomRows(delta * KinematicT::numControlSignals()).setZero();
    }
}

template <typename KinematicT>
void MPPIOptimizer<KinematicT>::setDistributionSigma(const size_t i, const float sigma)
{
    if (KinematicT::numControlSignals() <= i)
    {
        return;
    }

    std::lock_guard lock(mtx_);
    params_.distribtutions[i] = std::normal_distribution<float>(0.0, sigma);
}


template <typename KinematicT>
size_t MPPIOptimizer<KinematicT>::numControlSignals() const
{
    return KinematicT::numControlSignals();
}

template <typename KinematicT>
void MPPIOptimizer<KinematicT>::reset()
{
    std::lock_guard lock(mtx_);
    RCLCPP_DEBUG(get_logger(), "Optimizer reset");
    control_history_.setZero();
    samples_.setZero();
    sequence_.setZero();
}

template <typename KinematicT>
rclcpp::Logger MPPIOptimizer<KinematicT>::get_logger()
{
    return node_->get_logger().get_child("optimizer");
}


template <typename KinematicT>
std::expected<State, Error> MPPIOptimizer<KinematicT>::predictNextState(const State& current, const ControlVector& control) const
{
    // Apply the kinematic state transition
    const StateVector state = kinematic_->getNextState(current.state, control);
    // Get the 2D pose delta
    Pose2D delta = kinematic_->getPoseDelta(state);
    // Get the new 3D pose
    SurfacePose p(current.pose, current.face.unwrap());
    if (auto result = model_->predict(p, delta))
    {
        p = result.value();
    }
    else
    {
        return std::unexpected(result.error());
    }
    
    State s;
    s.state = Eigen::VectorXf(state);
    s.pose = p.pose;
    s.face = p.face;
    return s;
}

} // namespace mesh_mppi

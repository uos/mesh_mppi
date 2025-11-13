#pragma once
#include <mesh_mppi/types/State.hpp>
#include <mesh_mppi/scoring/CostFunction.hpp>
#include <mesh_mppi/Model.hpp>
#include <mesh_mppi/Error.hpp>
#include <mesh_mppi/Smoother.hpp>
#include <mesh_mppi/optimizer/OptimizerBase.hpp>

#include <mesh_map/mesh_map.h>

#include <std_msgs/msg/float32.hpp>

namespace mesh_mppi
{

template <typename KinematicT>
class MPPIOptimizer: public OptimizerBase
{
public:
    // Typedefinitions
    using ControlVector = typename KinematicT::ControlVector;
    using StateVector = typename KinematicT::StateVector;

    struct Result;

    bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name,
        const std::shared_ptr<CostFunction>& costfunction,
        const std::shared_ptr<KinematicT>& kinematic,
        const mesh_map::MeshMap::Ptr& map,
        const int32_t samples,
        const int32_t horizon
    );

    /**
     *  @brief Optimize the control signal based on an initial state
     *
     *  @param initial The initial state from which to optimize the control signal
     */
    std::expected<Result, Error> computeOptimalControl(const State& initial);

    /**
     *  @brief Set the number of samples to use in the optimization
     */
    void setNumSamples(size_t n);

    /**
     *  @brief Set the number of elements per sampled trajectory
     */
    void setPredictionHorizon(size_t n);

    /**
     *  @brief Set the standard deviation of the noise sampling distribution for control signal i.
     */
    void setDistributionSigma(const size_t i, const float sigma);

    /**
     *  @brief Get the number of control signals used by this optimizer (Equal to Kinematic control signals)
     */
    size_t numControlSignals() const;

    /**
     *  @brief Reset the internal state of the Optimizer, clear control history, samples, etc.
     */
    void reset();

protected:

    /**
     *  @brief Allows the AsyncOptimizer to access the logger
     */
    inline rclcpp::Logger get_logger();

private:

    void rolloutTrajectories(const State& initial, std::vector<Trajectory>& traj_out, std::vector<bool>& invalid_out);

    /**
     *  @brief Update the OptimalControl sequence using a weighted average.
     */
    void updateOptimalControlSequence(const Eigen::ArrayXXf& U, const Eigen::ArrayXf& weights);

    /**
     *  @brief  Shift the control sequence by 1 timestep
     */
    void shiftControlSequence();

    void publishTrajectorySamples(const std::vector<Trajectory>& trajectories, const std::vector<bool>& lethal);
    
    /// Use the Kinematic and Model to predict the next state
    inline std::expected<State, Error> predictNextState(const State& current, const ControlVector& control) const;

    rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter>& parameters);

private:
    // Mutex for thread safe access
    std::recursive_mutex mtx_;

    // The parameters
    using NDist = std::normal_distribution<float>;
    struct {
        int32_t samples;
        int32_t horizon;
        std::array<NDist, KinematicT::numControlSignals()> distribtutions;
        float temperature;
        float gamma;
    } params_;

    // ROS Node
    rclcpp::Node::SharedPtr node_;

    // Reconfigure callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Namespace for parameters
    std::string namespace_;

    // Publisher for the samples
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

    // Publisher for trajectory cost
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cost_pub_;

    // The cost function
    std::shared_ptr<CostFunction> cost_function_;

    // The kinematic
    std::shared_ptr<KinematicT> kinematic_;

    // The map
    mesh_map::MeshMap::Ptr map_;

    // The Model
    SurfaceModel::UniquePtr model_;

    // The current optimal control sequence
    Eigen::ArrayXf sequence_;

    // The current samples
    Eigen::ArrayXXf samples_;

    // The current trajectories
    std::vector<Trajectory> trajectories_;

    // The current scores
    Eigen::ArrayXf scores_;

    // The last computed optimal control
    ControlVector prev_control_;
    // The last 4 control commands
    Eigen::Array<float, 4, KinematicT::numControlSignals()> control_history_;

    // Random engine
    std::default_random_engine rand_;

    // Trajectory smoother
    Smoother<KinematicT::numControlSignals()> smoother_;

};

template <typename KinematicT>
struct MPPIOptimizer<KinematicT>::Result
{
    ControlVector control;
    Eigen::ArrayXf sequence;
    Trajectory trajectory;
};


} // namespace mesh_mppi

#include "MPPIOptimizer.inl"

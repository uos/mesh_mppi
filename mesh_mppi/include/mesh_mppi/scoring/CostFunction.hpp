#pragma once
#include <mesh_mppi/types/State.hpp>
#include <mesh_mppi/Error.hpp>
#include <mesh_mppi/scoring/XTECostFunction.hpp>

#include <mesh_map/mesh_map.h>

#include <rclcpp/node.hpp>
#include <nav_msgs/msg/path.hpp>

#include <expected>

namespace mesh_mppi
{

namespace v1
{
class CostFunction
{
public:

    CostFunction(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name,
        const mesh_map::MeshMap::Ptr& map
    );

    /**
     *  @brief Set the plan to be used for scoring
     */
    inline void set_plan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
    {
        std::lock_guard guard(mutex_);
        plan_ = plan;
    }

    /**
     *  @brief Called by the optimizer before scoring begins.
     *
     *  Extracts the next few meters from the plan to have a constant length plan
     *  during scoring.
     */
    void prepare_for_scoring(const State& initial, int prediction_horizon);
    
    /**
     *  @brief Calculates the cost of a trajectory
     */
    [[nodiscard]]
    std::expected<float, Error> score(
        const Eigen::VectorXf& controls,
        const Trajectory& traj
    ) const noexcept;

private:

    rcl_interfaces::msg::SetParametersResult reconfigure_callback(
        std::vector<rclcpp::Parameter> parameters
    );

    // ROS Node for parameters
    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
    std::string name_;

    // Synchronize set_plan and prepare_for_scoring
    std::mutex mutex_;

    // The map
    mesh_map::MeshMap::Ptr map_;
    
    // The current plan
    std::vector<geometry_msgs::msg::PoseStamped> plan_;

    // The target speed
    double target_speed_;

    // The reference for scoring;
    std::vector<Pose> ref_;

    // Weights
    using CostVector = Eigen::Vector<float, 6>;
    CostVector weights_;
    // The goal translation tolerance
    float goal_tolerance_;
};
} // Namespace v1

// The CostFunction to use during optimization
using CostFunction = XTECostFunction;

};

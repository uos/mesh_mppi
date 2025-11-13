#pragma once

#include <mesh_mppi/types/State.hpp>
#include <mesh_mppi/types/Line.hpp>
#include <mesh_mppi/Error.hpp>

#include <mesh_map/mesh_map.h>

#include <nav_msgs/msg/path.hpp>

#include <expected>

#include <rclcpp/parameter.hpp>

namespace mesh_mppi
{

/// A CostFunction which uses the Cross-Track-Error to determine distance to the path
class XTECostFunction
{
public:

    using Costs = Eigen::Array<float, 3, 1>;

    XTECostFunction(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name,
        const mesh_map::MeshMap::Ptr& map
    );

    void set_plan(const std::vector<geometry_msgs::msg::PoseStamped>& plan);

    // Called by the Optimizer before the scoring begins
    void prepare_for_scoring(const State& initial, int prediction_horizon, const std::vector<Trajectory>& trajectories);

    [[nodiscard]]
    std::expected<float, Error> score(
        const Eigen::VectorXf& controls,
        const Trajectory& traj
    ) const noexcept;

private:
    rcl_interfaces::msg::SetParametersResult reconfigure_callback(
        const std::vector<rclcpp::Parameter>& parameters
    );

private:
    std::mutex mutex_;
    std::vector<geometry_msgs::msg::PoseStamped> plan_;
    PolyLine plan_polyline_;
    // MeshMap to pull the map costs from
    mesh_map::MeshMap::Ptr map_;
    // The ROS Node
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
    // The parameter namespace of the controller plugin
    std::string name_;
    // The parameter namespace of this costfunction
    std::string namespace_;
    // The parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    // Subsegment of the path used during scoring
    PolyLine ref_;
    // Length of the ref_ path subsection
    float lookahead_;
    // Min/Max distance to the goal along the path
    float closest_distance_along_path_;
    float furthest_distance_along_path_;
    // Distances of the ref_ segments to the goal
    std::vector<float> dists_;
    float dist_to_go_;
    // The end of the current path
    Pose goal_;
    // Weights of the different cost terms
    Costs weights_;
    // The current configured goal_translation_tolerance
    float goal_tolerance_;
    // The cost limit above which a state is considered lethal
    float map_cost_limit_;
};

} // namespace mesh_mppi

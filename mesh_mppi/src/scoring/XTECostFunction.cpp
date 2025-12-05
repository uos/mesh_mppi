#include <limits>
#include <mesh_mppi/scoring/XTECostFunction.hpp>
#include <mesh_mppi/Util.hpp>

#include <mesh_map/util.h>

using mesh_map::toVector;

using std::placeholders::_1;

namespace mesh_mppi
{

XTECostFunction::XTECostFunction(
    const rclcpp::Node::SharedPtr& node,
    const std::string& name,
    const mesh_map::MeshMap::Ptr& map
)
{
    map_ = map;
    node_ = node;
    name_ = name;
    namespace_ = name + ".cost";
    weights_.setZero();
    
    // Setup parameters
    callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&XTECostFunction::reconfigure_callback, this, _1));

    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = namespace_ + ".xte_weight";
        desc.description = "The weight of the Cross-Track-Error. This keeps the robot closer to the path.";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 10.0;
        desc.set__floating_point_range({range});
        node_->declare_parameter<double>(desc.name, desc);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = namespace_ + ".goal_weight";
        desc.description = "The weight of the progress along the path. The sampled trajectory with the most progress along the path will have cost 0. All other trajectories cost is the distance to the closest point on the path reached by the furthest trajectory. +angular distance to the goal orientation  / PI if inside the translation goal tolerance, 1 otherwise.";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 10.0;
        desc.set__floating_point_range({range});
        node_->declare_parameter<double>(desc.name, desc);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = namespace_ + ".map_weight";
        desc.description = "The weight of the Map Error. This keeps the robot away from higher cost areas.";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 10.0;
        desc.set__floating_point_range({range});
        node_->declare_parameter<double>(desc.name, desc);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = namespace_ + ".lookahead_distance";
        desc.description = "The length of the segment of the path in front of the robot used for Cross-Track-Error calculation. Cannot be less than 2.0 meters.";
        node_->declare_parameter<double>(desc.name, 4.0, desc);
    }

    // Read the mbf dist_tolerance
    node->declare_parameter<double>("dist_tolerance");
    goal_tolerance_ = node->get_parameter("dist_tolerance").as_double();

    // Read the controller map_cost_limit
    map_cost_limit_ = node->get_parameter(name + ".map_cost_limit").as_double();

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliable();
    qos.keep_last(1);
    ref_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/local_plan", qos);
}

rcl_interfaces::msg::SetParametersResult XTECostFunction::reconfigure_callback(
    const std::vector<rclcpp::Parameter>& parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    for (const auto& param: parameters)
    {
        if (namespace_ + ".xte_weight" == param.get_name())
        {
            weights_(0) = param.get_value<double>();
        }
        else if (namespace_ + ".goal_weight" == param.get_name())
        {
            weights_(1) = param.get_value<double>();
        }
        else if (namespace_ + ".map_weight" == param.get_name())
        {
            weights_(2) = param.get_value<double>();
        }
        else if (namespace_ + ".lookahead_distance" == param.get_name())
        {
            const float value = param.get_value<double>();
            if (value <= 2.0)
            {
                result.set__successful(false);
                result.set__reason("Parameter 'lookahead_distance' cannot be less than 2.0 meters!");
                break;
            }
            lookahead_ = value;
        }
        // Controllers map_cost_limit is not in the .cost subnamespace
        else if (name_ + ".map_cost_limit" == param.get_name())
        {
            map_cost_limit_ = param.get_value<double>();
        }
        else if ("dist_tolerance" == param.get_name())
        {
            goal_tolerance_ = param.get_value<double>();
        }
    }

    return result;
}

void XTECostFunction::set_plan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
    std::lock_guard guard(mutex_);
    plan_ = plan;
    plan_polyline_.clear();
    // Create the polyline once here instead of each iteration in prepare_for_scoring
    for (auto it = plan_.begin(), next = std::next(plan_.begin()); next != plan_.end(); it++, next++)
    {
        plan_polyline_.push_back(
            Line(
                toVector(it->pose.position),
                toVector(next->pose.position)
            )
        );
    }
};

void XTECostFunction::prepare_for_scoring(const State& initial, int prediction_horizon, const std::vector<Trajectory>& trajectories)
{
    (void) prediction_horizon;
    std::lock_guard guard(mutex_);
    // Find the next x meters on the path in front and behind the robot
    ref_.clear();
    dists_.clear();
    goal_ = Pose::fromMsg(plan_.back().pose);

    // Compute the poses distance to the goal along the path
    std::vector<float> dists(plan_.size());
    auto out = dists.rbegin();
    *(out++) = 0.0; // The last pose is the goal so its distance is 0
    for (auto a = plan_.rbegin(), b = plan_.rbegin() + 1; b != plan_.rend(); ++a, ++b)
    {
        const Pose ap = Pose::fromMsg(a->pose);
        const Pose bp = Pose::fromMsg(b->pose);
        *(out++) = *std::prev(out) + ap.position.distance(bp.position);
    }

    // 1. Find the closest path segment
    size_t closest_segment = std::distance(
        plan_polyline_.cbegin(),
        plan_polyline_.closest_segment_to(initial.pose.position)
    );
    dist_to_go_ = dists.front();

    // 2. Build the local plan
    float length = 0.0f;
    size_t end_idx = closest_segment + 1;
    for (; end_idx < plan_.size(); end_idx++)
    {
        const Vector a = toVector(plan_[end_idx - 1].pose.position);
        const Vector b = toVector(plan_[end_idx].pose.position);
        
        if (end_idx == closest_segment + 1)
        {
            const Line segment(a, b);
            const Vector closest_point = segment.closest_point_to(initial.pose.position);
            length += closest_point.distance(b);
            ref_.push_back(Line(closest_point, b));
            dists_.push_back(dists[end_idx]);
        }
        else
        {
            const float seglen = a.distance(b);
            if (length + seglen < lookahead_)
            {
                ref_.push_back(Line(a, b));
                dists_.push_back(dists[end_idx]);
                length += seglen;
            }
            else
            {
                // Interpolate the last segment
                const Vector new_b = a + (b - a).normalized() * (lookahead_ - length);
                ref_.push_back(Line(a, new_b));
                dists_.push_back(dists[end_idx] - (b - new_b).length());
                break;
            }
        }
    }

    if (ref_pub_->get_subscription_count() > 0)
    {
        // Publish the local plan for debug
        auto msg = std::make_unique<nav_msgs::msg::Path>();
        msg->header.frame_id = map_->getGlobalFrameID();
        msg->header.stamp = node_->get_clock()->now();

        for (const auto& line: ref_)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header = msg->header;
            Pose pose(line.b(), Quaternion::Identity());
            p.pose = Pose::toMsg(pose);
            msg->poses.push_back(p);
        }

        ref_pub_->publish(std::move(msg));
    }

    // Determine which trajectory progresses the furthest along the path and set the local goal point
    closest_distance_along_path_ = std::numeric_limits<float>::infinity();
    furthest_distance_along_path_ = 0.0f;
    for(const auto& traj: trajectories)
    {
        if (traj.empty())
        {
            continue;
        }
        auto segment_it = ref_.closest_segment_to(traj.back().pose.position);
        auto cp = segment_it->closest_point_to(traj.back().pose.position);
        float dist_to_goal = dists_[std::distance(ref_.cbegin(), segment_it)]
            + cp.distance(segment_it->b())
            + traj.back().pose.position.distance(cp);

        closest_distance_along_path_ = std::min(closest_distance_along_path_, dist_to_goal);
        furthest_distance_along_path_ = std::max(furthest_distance_along_path_, dist_to_goal);
    }
}

std::expected<float, Error> XTECostFunction::score(
        const Eigen::VectorXf& controls,
        const Trajectory& traj
    ) const noexcept
{
    (void) controls;

    if (traj.empty())
    {
        return std::unexpected(Error::INVALID_INPUT);
    }

    Costs costs = Costs::Zero();
    
    // Cross-Track-Error
    for (const auto& state: traj)
    {
        costs(0) += ref_.closest_point_to(state.pose.position).distance2(state.pose.position);
    }
    costs(0) /= traj.size();

    // Distance to the goal along the path
    const auto line = ref_.closest_segment_to(traj.back().pose.position);
    const size_t idx = std::distance(ref_.cbegin(), line);
    const Vector cp = line->closest_point_to(traj.back().pose.position);
    // If we didn't include the distance from the trajectory to the plan the robot would not reach the goal
    const float dist_along_path = dists_[idx] + cp.distance(line->b()) + traj.back().pose.position.distance(cp);

    // Normalize the costs by range. If we did not do this the cost range differs depending on the
    // velocities/traveled distance
    float range = std::abs(furthest_distance_along_path_ - closest_distance_along_path_);
    if (range < std::numeric_limits<float>::epsilon())
    {
        range = 1.0f;
    }
    // NOTE: There is one edge case where this cost can be < 0:
    // The final optimized control sequence is not included in the range computed
    // during preparation. Therefore when we subtract the min it can happen that
    // this cost term is slightly negative. We choose to ignore this since the observed
    // absolute values were smaller than 0.001 which is okay for purposes of progress
    // checks and visualization.
    costs(1) = (dist_along_path - closest_distance_along_path_) / range;

    // Make the robot orient towards the goal orientation if the translation tolerance is reached.
    if (dist_along_path < goal_tolerance_)
    {
        costs(1) += goal_.orientation.angularDistance(traj.back().pose.orientation) * M_1_PIf;
    }
    else
    {
        costs(1) += 1.0;
    }

    // Map costs
    for (const auto& s: traj)
    {
        // We cannot use the maps barycentric coordinates because
        // a lot of times the point is on the edge or slightly outside
        // the triangle and in this cases the maps function failes.
        const auto& vertices = map_->mesh()->getVertexPositionsOfFace(s.face.unwrap());
        const auto& bary_coords = barycentric_coordinates(s.pose.position, vertices);
        float cost = map_->costAtPosition(
            map_->mesh()->getVerticesOfFace(s.face.unwrap()),
            bary_coords
        );

        if (std::isinf(cost) || std::isnan(cost))
        {
            // Lethals are sometimes set to infinity (depends on the top level layer)
            return std::unexpected(Error::LETHAL_COST);
        }
        else
        {
            costs(2) += cost;
        }

        if (map_cost_limit_ <= cost)
        {
            return std::unexpected(Error::LETHAL_COST);
        }
    }
    costs(2) /= traj.size();

    return (costs * weights_).sum();
}

} // namespace mesh_mppi

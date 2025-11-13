#include <mesh_mppi/scoring/CostFunction.hpp>
#include <mesh_mppi/Util.hpp>

namespace mesh_mppi
{
namespace v1
{
CostFunction::CostFunction(
    const rclcpp::Node::SharedPtr& node,
    const std::string& name,
    const mesh_map::MeshMap::Ptr& map
)
: node_(node)
, name_(name)
, map_(map)
{
    using rcl_interfaces::msg::ParameterDescriptor;
    using rcl_interfaces::msg::FloatingPointRange;
    std::map<std::string, std::pair<double, ParameterDescriptor>> parameters;

    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.translation_weight";
        descriptor.description = "The weight of the translation error";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {1.0, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.orientation_weight";
        descriptor.description = "The weight of the orientation error";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.05, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.control_weight";
        descriptor.description = "The weight of the control signal cost";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.1, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.velocity_weight";
        descriptor.description = "The weight of the angular velocity cost";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.1, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.map_weight";
        descriptor.description = "The weight of the map costs";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.3, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "cost.goal_weight";
        descriptor.description = "The weight of the goal cost";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.2, descriptor}}
        );
    }
    {
        ParameterDescriptor descriptor;
        descriptor.name = "target_speed";
        descriptor.description = "The target velocity in the xy plane";
        descriptor.floating_point_range.push_back(
            FloatingPointRange().set__from_value(0.0).set__to_value(10.0)
        );
        parameters.insert(
            {descriptor.name, {0.5, descriptor}}
        );
    }

    dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(
        &CostFunction::reconfigure_callback, this, std::placeholders::_1
    ));

    node->declare_parameters(name, parameters);

    // Read the mbf dist_tolerance
    node->declare_parameter<double>("dist_tolerance");
    goal_tolerance_ = node->get_parameter("dist_tolerance").as_double();

    // Publisher for the reference trajectory
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliable();
    qos.keep_last(1);
    ref_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/local_plan", qos);
}


rcl_interfaces::msg::SetParametersResult CostFunction::reconfigure_callback(
    std::vector<rclcpp::Parameter> parameters
)
{
    std::lock_guard guard(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;


    for (const auto& param: parameters)
    {
        if (name_ + ".cost.translation_weight" == param.get_name())
        {
            weights_(0) = param.as_double();
        }
        else if (name_ + ".cost.orientation_weight" == param.get_name())
        {
            weights_(1) = param.as_double();
        }
        else if (name_ + ".cost.control_weight" == param.get_name())
        {
            weights_(2) = param.as_double();
        }
        else if (name_ + ".cost.velocity_weight" == param.get_name())
        {
            weights_(3) = param.as_double();
        }
        else if (name_ + ".cost.map_weight" == param.get_name())
        {
            weights_(4) = param.as_double();
        }
        else if (name_ + ".cost.goal_weight" == param.get_name())
        {
            weights_(5) = param.as_double();
        }
        else if (name_ + ".target_speed" == param.get_name())
        {
            target_speed_ = param.as_double();
        }
        else if ("dist_tolerance" == param.get_name())
        {
            goal_tolerance_ = param.as_double();
        }
    }

    return result;
}


void CostFunction::prepare_for_scoring(const State& initial, int prediction_horizon)
{
    std::lock_guard guard(mutex_);
    // The plan should never be empty when this function is called!
    if (plan_.empty())
    {
        ref_.resize(prediction_horizon);
        std::fill(ref_.begin(), ref_.end(), initial.pose);
        return;
    }
    
    // If only one pose in the plan thats it
    if (plan_.size() == 1)
    {
        ref_.resize(prediction_horizon);
        std::fill(ref_.begin(), ref_.end(), Pose::fromMsg(plan_.back().pose));
        return;
    }

    // Find the closest segment
    double min_dist = std::numeric_limits<double>::infinity();
    // Needed info
    size_t seg_a = 0;
    size_t seg_b = 1;
    Vector closest;
    for (auto i = 1u; i < plan_.size(); i++)
    {
        const auto a = Vector(
            plan_[i - 1].pose.position.x,
            plan_[i - 1].pose.position.y,
            plan_[i - 1].pose.position.z
        );
        const auto b = Vector(
            plan_[i].pose.position.x,
            plan_[i].pose.position.y,
            plan_[i].pose.position.z
        );

        // Distance to a line segment
        // Segment vector
        auto ab = b - a;
        // Segment start to position
        auto ap = initial.pose.position - a;
        
        // Scalar projection of ap onto ab
        double t = ap.dot(ab) / ab.length2();
        t = std::clamp(t, 0.0, 1.0);

        // Closest point on the segment
        Vector p = a + ab * t;

        // Squared distance to segment
        double dist = (initial.pose.position - p).length2();
        
        if (dist < min_dist)
        {
            min_dist = dist;
            seg_a = i - 1;
            seg_b = i;
            closest = p;
        }
    }
    
    // Sample the reference trajectory
    std::vector<Pose> res;
    Vector position = closest;
    const float v_ref = target_speed_;

    for (int i = 0; i < prediction_horizon; i++)
    {
        // End of path
        if (seg_b >= plan_.size())
        {
            res.push_back(Pose::fromMsg(plan_.back().pose));
            continue;
        }

        auto seg_beg = Vector(
            plan_[seg_a].pose.position.x,
            plan_[seg_a].pose.position.y,
            plan_[seg_a].pose.position.z
        );

        auto seg_end = Vector(
            plan_[seg_b].pose.position.x,
            plan_[seg_b].pose.position.y,
            plan_[seg_b].pose.position.z
        );
        Vector segment_vec = seg_end - seg_beg;

        // Move along the path
        float dist = v_ref * 0.05;
        float rem_segment_len = segment_vec.length() - (position - seg_beg).length();
        
        while (dist > rem_segment_len)
        {
            position = seg_end;
            dist -= rem_segment_len;
            seg_a = seg_b;
            seg_b++;

            // Reached end of plan
            if (seg_b >= plan_.size())
            {
                break;
            }

            seg_beg = Vector(
                plan_[seg_a].pose.position.x,
                plan_[seg_a].pose.position.y,
                plan_[seg_a].pose.position.z
            );

            seg_end = Vector(
                plan_[seg_b].pose.position.x,
                plan_[seg_b].pose.position.y,
                plan_[seg_b].pose.position.z
            );

            segment_vec = seg_end - position;
            rem_segment_len = segment_vec.length();
        }
        
        if (seg_b >= plan_.size())
        {
            res.push_back(Pose::fromMsg(plan_.back().pose));
            continue;
        }
        else
        {
            // Interpolate pose and orientation
            position += segment_vec.normalized() * dist;
            Pose pose_a = Pose::fromMsg(plan_[seg_a].pose);

            res.push_back(Pose{position, pose_a.orientation});
        }
    }

    ref_ = std::move(res);

    {
        auto msg = std::make_unique<nav_msgs::msg::Path>();
        msg->header.frame_id = map_->getGlobalFrameID();
        msg->header.stamp = node_->get_clock()->now();

        for (const auto& pose: ref_)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header = msg->header;
            p.pose = Pose::toMsg(pose);
            msg->poses.push_back(p);
        }

        ref_pub_->publish(std::move(msg));
    }
}


std::expected<float, Error> CostFunction::score(
    const Eigen::VectorXf& controls,
    const Trajectory& traj
) const noexcept
{
    CostVector costs = CostVector::Zero();

    if (traj.size() != ref_.size())
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Reference trajectory and trajectory differ in length. %ld vs %ld", ref_.size(), traj.size()
        );
        return std::unexpected(Error::TRAJECTORY_REFERENCE_LENGTH_MISSMATCH);
    }
    
    // Average translation error to the reference trajectory
    // Range [0, INF] (In theory the error can climb infinitly)
    // Average rotation error to the reference trajectory
    // Range [0, PI]
    for (auto i = 0u; i < traj.size(); i++)
    {
        costs(0) += (ref_[i].position - traj[i].pose.position).length2();
        costs(1) += ref_[i].orientation.angularDistance(traj[i].pose.orientation);
    }
    costs(0) /= traj.size();
    costs(1) /= traj.size();
    costs(1) *= M_1_PI;
    
    // Prefer low control signals
    for (int t = 0; t < (controls.rows() / 2); t++)
    {
        costs(2) += controls.middleRows<2>(t * 2).squaredNorm();
    }
    costs(2) /= traj.size();

    // Prefer low angular velocities to prevent oscillations
    for (const auto& s: traj)
    {
        costs(3) += std::abs(s.state(1)) + 0.5 * std::abs(s.state(0));
    }
    costs(3) /= traj.size();

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
            // Lethals are set to infinity
            return std::unexpected(Error::LETHAL_COST);
        }
        else
        {
            costs(4) += cost;
        }
        // Inscribed radius cost is 0.8 This should probably be a parameter too
        if (0.8 <= cost)
        {
            return std::unexpected(Error::LETHAL_COST);
        }
    }
    costs(4) /= traj.size();

    //=== Goal cost ===
    // Punish rotation errors, but only when in range of the goal (in translation_tolerance)
    // if we always punish the orientation error the robot might not be able to decrease the
    // translation error
    //
    // Observation: 
    //  - The rotational error is bounded to [0, PI]
    //  - The translational error is bounded [0, inf]
    const float o_err = ref_.back().orientation.angularDistance(traj.back().pose.orientation);
    const float t_err = ref_.back().position.distanceFrom(traj.back().pose.position);
    
    costs(5) = t_err;
    if (t_err < goal_tolerance_)
    {
        costs(5) += o_err * M_1_PI;
    }
    else
    {
        costs(5) += 1.0;
    }
    
    return costs.dot(weights_);
}
} // namespace v1
} // namespace mesh_mppi

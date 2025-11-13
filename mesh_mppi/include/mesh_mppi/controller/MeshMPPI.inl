#pragma once

#include <mesh_mppi/controller/MeshMPPI.hpp>
#include <mesh_mppi/Util.hpp>

#include <mbf_msgs/action/exe_path.hpp>

#include <mesh_map/util.h>

namespace mesh_mppi
{

using ExePathResult = mbf_msgs::action::ExePath::Result;

template <typename KinematicsT>
uint32_t MeshMPPI<KinematicsT>::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::TwistStamped& velocity,
    geometry_msgs::msg::TwistStamped& cmd_vel,
    std::string& message
)
{
    State current;
    current.pose = Pose::fromMsg(pose.pose);
    current.state = Eigen::VectorXf(kinematics_->getInitialStateFromVelocity(velocity.twist));

    // Store position and orientation for goal check
    this->setCurrentPoseAndVelocity(pose.pose, velocity.twist);
    
    // Project the position onto the face the robot is currently on
    if (this->updateCurrentFace(current))
    {
        // Make sure pos is on the surface
        current.face = getCurrentFace();
        std::array<float, 3> bary;
        float dist = 0.0;
        if (!map_->projectedBarycentricCoords(
            current.pose.position,
            current.face.unwrap(),
            bary,
            dist
        ))
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not project current position to map");
            return ExePathResult::OUT_OF_MAP;
        }

        current.pose.position = mesh_map::linearCombineBarycentricCoords(
            map_->mesh()->getVertexPositionsOfFace(current.face.unwrap()),
            bary
        );

        // FIXME: Do we really have to check this or do we trust the MeshMap?
        // Maybe this can be removed when the MeshMap uses a BVH for closest point?
        if (0.001 < point_to_plane_distance(
            current.pose.position,
            map_->mesh()->getVertexPositionsOfFace(current.face.unwrap())[0],
            map_->faceNormals()[current.face.unwrap()]
        ))
        {
            message = "Projected robot position is not on the mesh surface!";
            return ExePathResult::OUT_OF_MAP;
        }

        // Check if we are in collision
        const float init_cost = map_->costAtPosition(map_->mesh()->getVerticesOfFace(current.face.unwrap()), bary);
        if (params_.map_cost_limit <= init_cost)
        {
            message = "Current robot position has inscribed radius cost!";
            return ExePathResult::COLLISION;
        }
    }
    else
    {
        message = "Out of map";
        return ExePathResult::OUT_OF_MAP;
    }

    // Get future from last computation
    auto result = std::move(future_);
    // Is only the case if the robot is not already moving
    if (first_)
    {
        first_ = false;
        future_ = optimizer_.computeOptimalControl(current);
        return ExePathResult::INTERNAL_ERROR;
    }
    else
    {
        // Wait for the previous computation to finish
        if (result.valid())
        {
            result.wait();
        }
    }
    
    // === Get the result of previous async prediction ===
    Trajectory trajectory;
    typename KinematicsT::ControlVector control;
    Eigen::ArrayXf sequence;
    if (result.valid())
    {
        const auto res = result.get();
        // Check for error during optimization
        if (!res.has_value())
        {
            // Reset the optimizer and start the next iteration
            optimizer_.reset();
            future_ = optimizer_.computeOptimalControl(current);
            // Handle the error
            const auto& error = res.error();
            if (Error::NO_VALID_COMMAND == error)
            {
                message = "No valid command found during optimization!";
                return ExePathResult::NO_VALID_CMD;
            }
            else if (Error::OUT_OF_MAP == error)
            {
                message = "The robot is outside the map!";
                return ExePathResult::OUT_OF_MAP;
            }
            else
            {
                message = std::format(
                    "An unhandled error occured during optimization! {}",
                    int(error)
                );
                return ExePathResult::INTERNAL_ERROR;
            }
        }

        trajectory = res->trajectory;
        control = res->control;
        sequence = res->sequence;
    }
    else
    {
        // Reset the optimizer if the computation did not finish in time
        optimizer_.reset();
        message = "Computation did not finish in time";
        return ExePathResult::NO_VALID_CMD;
    }

    // === Start the next async prediction ===
    future_ = optimizer_.computeOptimalControl(current);
    
    cmd_vel.twist = kinematics_->getTwistFromControls(current, control);
    cmd_vel.header.stamp = node_->get_clock()->now();

    this->publishOptimalTrajectory(trajectory);
    this->publishOptimalControlSequence(sequence, params_.horizon, optimizer_.numControlSignals(), cmd_vel.header.stamp);

    return 0;
}

template <typename KinematicsT>
bool MeshMPPI<KinematicsT>::initialize()
{
    kinematics_ = std::make_shared<KinematicsT>();

    if (!kinematics_)
    {
        RCLCPP_ERROR(node_->get_logger(), "MeshMPPI: make_unique<KinematicT> returned nullptr");
        return false;
    }
    else
    {
        // C++ hiding rule means the compiler cannot find the base class method because initialize() exists in derived classes
        if (!kinematics_->KinematicsBase::initialize(node_, name_, 1.0f / params_.controller_frequency))
        {
            return false;
        }
    }

    this->optimizer_.initialize(node_, name_, cost_function_, kinematics_, map_, params_.samples, params_.horizon);

    return true;
}

} // namespace mesh_mppi

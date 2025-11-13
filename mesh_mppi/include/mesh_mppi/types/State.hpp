#pragma once

#include <mesh_mppi/types/Vector.hpp>
#include <mesh_mppi/types/Pose.hpp>

namespace mesh_mppi
{

/**
 * @brief   Stores the state of the system.
 */
struct State
{
    inline State()
    {
        pose.position = Vector(0, 0, 0);
        pose.orientation.setIdentity();
        state.setZero();
    }
    State(const State& other) = default;
    State& operator=(const State& other) = default;
    bool operator==(const State& other) const = default;

    Eigen::VectorXf state;
    Pose pose;
    lvr2::OptionalFaceHandle face;
};

using Trajectory = typename std::vector<State>;

} // namespace mesh_mppi

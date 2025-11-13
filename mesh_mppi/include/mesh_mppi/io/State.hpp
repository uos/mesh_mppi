#pragma once

#include "Serialization.hpp"
#include <mesh_mppi/io/Pose.hpp>
#include <mesh_mppi/types/State.hpp>

namespace mesh_mppi
{
namespace io
{

// TODO: This could be in source file if we remove inline, would compile faster
template<>
inline void write<mesh_mppi::State>(std::ostream& out, const mesh_mppi::State& elem)
{
    // Write face_idx
    const uint32_t fidx = elem.face.unwrap().idx();
    out.write(reinterpret_cast<const char*>(&fidx), sizeof(uint32_t));

    // Write pose
    write(out, elem.pose);

    // Write state
    const uint32_t n = elem.state.rows();
    out.write(reinterpret_cast<const char*>(&n), sizeof(uint32_t));
    // I dont know if we could just read elem.state.data(), might be problematic because Eigen...
    for (uint32_t i = 0; i < n; i++)
    {
        out.write(reinterpret_cast<const char*>(&elem.state(i)), sizeof(float));
    }
}

template<>
inline void read<mesh_mppi::State>(std::istream& in, mesh_mppi::State& elem)
{
    // Read face_idx
    uint32_t fidx = 0;
    in.read(reinterpret_cast<char*>(&fidx), sizeof(uint32_t));
    elem.face = lvr2::FaceHandle(fidx);

    // Read pose
    read(in, elem.pose);

    // Read state
    uint32_t n = 0;
    in.read(reinterpret_cast<char*>(&n), sizeof(uint32_t));
    elem.state = Eigen::VectorXf::Zero(n);
    for (uint32_t i = 0; i < n; i++)
    {
        float tmp = 0;
        in.read(reinterpret_cast<char*>(&tmp), sizeof(float));
        elem.state(i) = tmp;
    }
}

} // namespace io
} // namespace mesh_mppi

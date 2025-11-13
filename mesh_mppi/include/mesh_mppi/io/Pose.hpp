#pragma once

#include "Serialization.hpp"
#include <mesh_mppi/types/Pose.hpp>

namespace mesh_mppi
{
namespace io
{

// TODO: This could be in source file if we remove inline, would compile faster
template<>
inline void write<mesh_mppi::Pose>(std::ostream& out, const mesh_mppi::Pose& elem)
{
    // Write position
    out.write(reinterpret_cast<const char*>(&elem.position.x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&elem.position.y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&elem.position.z), sizeof(float));
    // Write orientation
    out.write(reinterpret_cast<const char*>(&elem.orientation.x()), sizeof(float));
    out.write(reinterpret_cast<const char*>(&elem.orientation.y()), sizeof(float));
    out.write(reinterpret_cast<const char*>(&elem.orientation.z()), sizeof(float));
    out.write(reinterpret_cast<const char*>(&elem.orientation.w()), sizeof(float));
}

template<>
inline void read<mesh_mppi::Pose>(std::istream& in, mesh_mppi::Pose& elem)
{
    // Read position
    in.read(reinterpret_cast<char*>(&elem.position.x), sizeof(float));
    in.read(reinterpret_cast<char*>(&elem.position.y), sizeof(float));
    in.read(reinterpret_cast<char*>(&elem.position.z), sizeof(float));
    // Read orientation
    in.read(reinterpret_cast<char*>(&elem.orientation.x()), sizeof(float));
    in.read(reinterpret_cast<char*>(&elem.orientation.y()), sizeof(float));
    in.read(reinterpret_cast<char*>(&elem.orientation.z()), sizeof(float));
    in.read(reinterpret_cast<char*>(&elem.orientation.w()), sizeof(float));
}


template<>
inline void write<mesh_mppi::SurfacePose>(std::ostream& out, const mesh_mppi::SurfacePose& elem)
{
    // Write face_idx
    const uint32_t fidx = elem.face.idx();
    out.write(reinterpret_cast<const char*>(&fidx), sizeof(uint32_t));

    // Write pose
    write(out, elem.pose);
}

template<>
inline void read<mesh_mppi::SurfacePose>(std::istream& in, mesh_mppi::SurfacePose& elem)
{
    // Read face_idx
    uint32_t fidx = 0;
    in.read(reinterpret_cast<char*>(&fidx), sizeof(uint32_t));
    elem.face = lvr2::FaceHandle(fidx);

    // Read pose
    read(in, elem.pose);
}

} // namespace io
} // namespace mesh_mppi

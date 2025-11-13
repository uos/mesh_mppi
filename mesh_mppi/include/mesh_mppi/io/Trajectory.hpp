#pragma once

#include "Serialization.hpp"
#include "State.hpp"

#include <mesh_mppi/types/State.hpp>

namespace mesh_mppi
{
namespace io
{

// TODO: This could be in source file if we remove inline, would compile faster
template<>
inline void write<mesh_mppi::Trajectory>(std::ostream& out, const mesh_mppi::Trajectory& elem)
{
    // Write length
    const size_t length = elem.size();
    out.write(reinterpret_cast<const char*>(&length), sizeof(size_t));

    // Write all states
    for (const State& state: elem)
    {
        write(out, state);
    }
}

template<>
inline void read<mesh_mppi::Trajectory>(std::istream& in, mesh_mppi::Trajectory& elem)
{
    // Read length
    size_t length = 0;
    in.read(reinterpret_cast<char*>(&length), sizeof(size_t));
    elem.clear();
    elem.resize(length);

    // Read states
    for (State& state: elem)
    {
        read(in, state);
    }
}

} // namespace io
} // namespace mesh_mppi

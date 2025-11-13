#pragma once

#include <ostream>

namespace mesh_mppi
{
namespace io
{

/**
*   @brief template to serialize a type.
*
*   @param out ostream to write to, must be for binary writing.
*/
template <typename T>
void write(std::ostream& out, const T& elem);

/**
*   @brief template to deserialize a type.
*
*   @param in istream to read from, must be for binary reading.
*/
template <typename T>
void read(std::istream& in, T& elem);

} // namespace io
} // namespace mesh_mppi

#pragma once

#include <mesh_map/mesh_map.h>

namespace mesh_mppi
{

// We use Vector Types from MeshMap (lvr2) and Eigen's Quaternion
using mesh_map::Vector;
using mesh_map::Normal;
using Quaternion = Eigen::Quaternionf;

} // namespace mesh_mppi

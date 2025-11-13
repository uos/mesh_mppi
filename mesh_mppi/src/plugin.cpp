#include <mesh_mppi/controller/MeshMPPI.hpp>
#include <mesh_mppi/kinematics/DifferentialDrive.hpp>
#include <mesh_mppi/kinematics/Bicycle.hpp>
#include <pluginlib/class_list_macros.hpp>

// Register plugins for different kinematics type
using DifferentialDrive = mesh_mppi::kinematics::DifferentialDriveKinematics;
PLUGINLIB_EXPORT_CLASS(mesh_mppi::MeshMPPI<DifferentialDrive>, mbf_mesh_core::MeshController);

using Bicycle = mesh_mppi::kinematics::BicycleKinematics;
PLUGINLIB_EXPORT_CLASS(mesh_mppi::MeshMPPI<Bicycle>, mbf_mesh_core::MeshController);

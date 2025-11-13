#include <mesh_mppi/kinematics/KinematicsBase.hpp>

namespace mesh_mppi
{

bool KinematicsBase::initialize(const rclcpp::Node::SharedPtr& node, const std::string& name, float time_delta)
{
    node_ = node;
    name_ = name + ".kinematics";
    this->setDeltaTime(time_delta);

    // Allow subclasses to init parameters etc.
    return this->initialize();
}

} // namespace mesh_mppi

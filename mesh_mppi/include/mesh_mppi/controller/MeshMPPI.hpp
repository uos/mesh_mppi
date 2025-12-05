#pragma once
#include <mesh_mppi/controller/MeshMPPIBase.hpp>

namespace mesh_mppi
{

template <typename KinematicsT>
class MeshMPPI: public MeshMPPIBase
{
public:

    virtual uint32_t computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::TwistStamped& velocity,
        geometry_msgs::msg::TwistStamped& cmd_vel,
        std::string& message
    ) override;

protected:

    // Initialize the Kinematics and the Optimizer
    virtual bool initialize() override; 

    virtual OptimizerBase& getOptimizerBase() override
    {
        return optimizer_;
    }

    virtual KinematicsBase& getKinematicsBase() override
    {
        return *kinematics_;
    }

    virtual void resetFuture() override
    {
        future_ = typename AsyncOptimizer<KinematicsT>::Future();
    }

private:
    // The robot kinematic model
    std::shared_ptr<KinematicsT> kinematics_;

    // The optimizer
    AsyncOptimizer<KinematicsT> optimizer_;

    // Future for the currently running optimization
    AsyncOptimizer<KinematicsT>::Future future_;
};


} // namespace mesh_mppi

#include "MeshMPPI.inl"

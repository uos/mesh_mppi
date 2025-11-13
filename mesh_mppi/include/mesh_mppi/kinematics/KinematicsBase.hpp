#pragma once

#include <rclcpp/node.hpp>


namespace mesh_mppi
{

/**
 * @brief Base interface for type erased initialization of Kinematics
 */
class KinematicsBase
{
public:

    /**
     * @brief Initialize the Kinematic parameters etc. Do not override this to init your Kinematics, use initialize() instead.
     *
     * @param node The rclcpp Node to register parameters with
     * @param name The namespace of the controller
     * @param time_delta The dt to use in forward state propagation
     */
    [[nodiscard]]
    virtual bool initialize(const rclcpp::Node::SharedPtr& node, const std::string& name, float time_delta);

    /**
     * @brief set the delta time to use in forward kinematics etc.
     * @param dt the time delta to use. Must be != 0.0
     */
    void setDeltaTime(float dt)
    {
        if (dt == 0.0)
        {
            throw std::invalid_argument("Kinematics::setDeltaTime(dt): dt must not be 0.0!");
        }
        delta_t_ = dt;
    }

    /**
     * @brief Get the time delta used by the kinematics
     */
    [[nodiscard]]
    float getDeltaTime() const 
    {
        return delta_t_;
    }

protected:
    /**
     * @brief Initialization callback for implementations. Override this to setup parameters etc.
     * This is called after node_ and name_ have beed set.
     */
    [[nodiscard]]
    virtual bool initialize() {return true;};

    rclcpp::Node::SharedPtr node_;
    // The namespace for parameters
    std::string name_;

private:
    // The time delta in seconds used when integrating times
    float delta_t_ = 1.0;
};

} // namespace mesh_mppi

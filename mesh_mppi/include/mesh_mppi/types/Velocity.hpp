#pragma once

#include <mesh_mppi/types/Vector.hpp>

#include <geometry_msgs/msg/twist.hpp>

namespace mesh_mppi
{

/**
 * @brief Represents a 3D velocity
 */
struct Velocity
{
    Vector linear;
    Vector angular;

    static Velocity fromMsg(const geometry_msgs::msg::Twist& msg)
    {
        Velocity vel;
        vel.linear.x = msg.linear.x;
        vel.linear.y = msg.linear.y;
        vel.linear.z = msg.linear.z;

        vel.angular.x = msg.angular.x;
        vel.angular.y = msg.angular.y;
        vel.angular.z = msg.angular.z;

        return vel;
    }

    static geometry_msgs::msg::Twist toMsg(const Velocity& msg)
    {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = msg.linear.x;
        vel.linear.y = msg.linear.y;
        vel.linear.z = msg.linear.z;

        vel.angular.x = msg.angular.x;
        vel.angular.y = msg.angular.y;
        vel.angular.z = msg.angular.z;

        return vel;
    }
};

} // namespace mesh_mppi

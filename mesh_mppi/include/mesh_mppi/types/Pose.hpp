#pragma once

#include <mesh_mppi/types/Vector.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace mesh_mppi
{

// Represents a 3D Pose
struct Pose
{
    Vector position;
    Quaternion orientation;

    static Pose fromMsg(const geometry_msgs::msg::Pose& pose)
    {
        return Pose(
            Vector(
                pose.position.x,
                pose.position.y,
                pose.position.z
            ),
            Quaternion(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            )
        );
    }

    static geometry_msgs::msg::Pose toMsg(const Pose& pose)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = pose.position.x;
        p.position.y = pose.position.y;
        p.position.z = pose.position.z;

        p.orientation.x = pose.orientation.x();
        p.orientation.y = pose.orientation.y();
        p.orientation.z = pose.orientation.z();
        p.orientation.w = pose.orientation.w();

        return p;
    }

    bool operator==(const Pose& other) const = default;

    Pose()
    : position(0.0, 0.0, 0.0)
    , orientation(Quaternion::Identity())
    {}

    Pose(const Vector& position, const Quaternion& orientation)
    : position(position)
    , orientation(orientation)
    {}

    Pose(const Pose& other) = default;

    Pose& operator=(const Pose& other) = default;
};

/// Represents a pose on a mesh surface
struct SurfacePose
{
    SurfacePose(const Pose& pose, const lvr2::FaceHandle& face)
    : pose(pose)
    , face(face)
    {}
    
    SurfacePose(const SurfacePose&) = default;
    SurfacePose& operator=(const SurfacePose&) = default;

    Pose pose;
    lvr2::FaceHandle face;
};

// Represents a 2D Pose
struct Pose2D
{
    Pose2D()
    : x(0.0f)
    , y(0.0f)
    , theta(0.0f)
    {}

    Pose2D(const Pose2D& other) = default;
    Pose2D& operator=(const Pose2D& other) = default;

    float x;
    float y;
    float theta;
};

} // namespace mesh_mppi

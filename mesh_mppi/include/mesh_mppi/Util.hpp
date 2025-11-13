#pragma once

#include <Eigen/Dense>
#include <optional>
#include <mesh_mppi/types/Vector.hpp>

namespace mesh_mppi
{

inline Vector to_map_vec(const Eigen::Vector3f& vec)
{
    return Vector(vec.x(), vec.y(), vec.z());
}

inline Eigen::Vector3f to_eigen_vec(const Vector& vec)
{
    return Eigen::Vector3f(vec.x, vec.y, vec.z);
}

/**
 *  @brief Get the other face if it exists
 */
inline std::optional<lvr2::FaceHandle> opposite_face(const lvr2::FaceHandle& faceH, const std::array<lvr2::OptionalFaceHandle, 2>& faces)
{
    if (faces[0] && faces[0].unwrap() != faceH)
    {
        return faces[0].unwrap();
    }
    else if (faces[1] && faces[1].unwrap() != faceH)
    {
        return faces[1].unwrap();
    }
    else
    {
        return std::nullopt;
    }
}

inline bool point_inside_triangle(const Vector& point, const Vector& a, const Vector& b, const Vector& c)
{
    // Triangle edges
    const Vector ab = b - a;
    const Vector bc = c - b;
    const Vector ca = a - c;

    // Vectors to the point
    const Vector ap = point - a;
    const Vector bp = point - b;
    const Vector cp = point - c;

    const Vector abx = ab.cross(ap);
    const Vector bcx = bc.cross(bp);
    const Vector cax = ca.cross(cp);

    if (abx.dot(bcx) > 0 && bcx.dot(cax) > 0 && cax.dot(abx) > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline bool point_inside_triangle(const Vector& point, const std::array<Vector, 3>& vertices)
{
    return point_inside_triangle(point, vertices[0], vertices[1], vertices[2]);
}

/**
 *  @brief Same as \ref mesh_map::barycentricCoords but without the inside checks.
 *
 *  We don't need to check if the point is inside the triangle because this is guaranteed
 *  by construction of the algorithm.
 */
inline const std::array<float, 3> barycentric_coordinates(const Vector& p, const std::array<Vector,3>& positions)
{
    std::array<float, 3> result{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}; // default: barycenter
    
    const Vector& v0 = positions[0];
    const Vector& v1 = positions[1];
    const Vector& v2 = positions[2];

    // compute plane's normal
    Vector v0v1 = v1 - v0;
    Vector v0v2 = v2 - v0;

    // no need to normalize
    Vector N = v0v1.cross(v0v2);  // N
    float denom = N.dot(N);

    // Step 2: inside-outside test
    Vector C;  // vector perpendicular to triangle's plane

    // edge 0
    Vector edge0 = v1 - v0;
    Vector vp0 = p - v0;
    C = edge0.cross(vp0);

    // edge 1
    Vector edge1 = v2 - v1;
    Vector vp1 = p - v1;
    C = edge1.cross(vp1);
    result[0] = N.dot(C);

    // edge 2
    Vector edge2 = v0 - v2;
    Vector vp2 = p - v2;
    C = edge2.cross(vp2);
    result[1] = N.dot(C);

    result[0] /= denom;
    result[1] /= denom;
    result[2] = 1 - result[0] - result[1];

    return result;
}

/**
*   @brief Determine the signed distance of a point to a plane.
*/
float signed_point_to_plane_distance(const Vector& point, const Vector& ref, const Normal& normal);

inline float point_to_plane_distance(const Vector& point, const Vector& ref, const Normal& normal)
{
    return std::abs(signed_point_to_plane_distance(point, ref, normal));
}

/**
*   @brief Project a point onto a plane
*/
Vector project_point_to_plane(const Vector& point, const Vector& ref, const Normal& normal);

} // namespace mesh_mppi

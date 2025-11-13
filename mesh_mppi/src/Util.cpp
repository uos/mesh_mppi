#include <mesh_mppi/Util.hpp>

namespace mesh_mppi
{

float signed_point_to_plane_distance(const Vector& point, const Vector& ref, const Normal& normal)
{
    const Vector tmp = point - ref;
    return normal.dot(tmp);
}

Vector project_point_to_plane(const Vector& point, const Vector& ref, const Normal& normal)
{
    const float dist = signed_point_to_plane_distance(point, ref, normal);

    return point - normal * dist;
}

} // namespace mesh_mppi

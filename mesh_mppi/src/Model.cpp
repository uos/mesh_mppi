#include <array>
#include <mesh_mppi/Model.hpp>
#include <mesh_mppi/Util.hpp>
#include <mesh_mppi/types/State.hpp>

#include <mesh_map/util.h>
#include <assert.h>

#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/geometry/PMPMesh.hpp>

namespace mesh_mppi
{

// The precision to use when checking distances.
// We do not need sub millimeter precision here.
constexpr float MM_PRECISION = 0.001;

MeshSurfaceModel::MeshSurfaceModel(
    const mesh_map::MeshMap::Ptr& map
)
: mesh_(map->mesh())
{
    assert(map != nullptr);
    assert(mesh_ != nullptr);
    using FaceNormalMap = lvr2::DenseFaceMap<Normal>;
    if (auto opt = map->meshIO()->getDenseAttributeMap<FaceNormalMap>("face_normals"))
    {
        normals_ = opt.value();
    }
    else
    {
        normals_ = lvr2::calcFaceNormals(*mesh_);
    }
}

MeshSurfaceModel::MeshSurfaceModel(
    const std::shared_ptr<lvr2::PMPMesh<mesh_map::Vector>>& mesh
)
: mesh_(mesh)
{
    assert(mesh != nullptr);
    normals_ = lvr2::calcFaceNormals(*mesh_);
}

std::expected<SurfacePose, Error> MeshSurfaceModel::predict(const SurfacePose& current, const Pose2D& delta) noexcept
{
    // Check if the pose is actually on the surface, its a precondition
    // for the move_on_surface_algorithm
    if (std::abs(signed_point_to_plane_distance(
        current.pose.position,
        mesh_->getVertexPositionsOfFace(current.face)[0],
        normals_[current.face]
    )) > MM_PRECISION)
    {
        return std::unexpected(Error::INVALID_INPUT);
    }

    const float delta_length = std::hypotf(delta.x, delta.y);
    const Quaternion delta_orientation(Eigen::AngleAxisf(delta.theta, Eigen::Vector3f::UnitZ()));

    // delta.position might be to small to be normalized or even 0
    if (MM_PRECISION > delta_length)
    {
        Pose pose;
        pose.position = current.pose.position;
        pose.orientation = current.pose.orientation * delta_orientation;
        return SurfacePose(pose, current.face);
    }

    // Move the position on the surface of the map
    if (auto opt = this->move_on_surface(
        current,
        to_map_vec(current.pose.orientation * Eigen::Vector3f(delta.x, delta.y, 0.0)),
        delta_length
    ))
    {
        SurfacePose pose = opt.value();

        // Apply the orientation delta
        pose.pose.orientation = pose.pose.orientation * delta_orientation;

        // Check that the resulting pose is on the surface
        if (point_to_plane_distance(
            pose.pose.position,
            mesh_->getVertexPositionsOfFace(pose.face)[0],
            normals_[pose.face]
        ) > MM_PRECISION)
        {
            return std::unexpected(Error::PREDICTION_ERROR);
        }

        return pose;
    }
    else
    {
        switch (opt.error())
        {
            case MoveError::OUT_OF_MAP:
                return std::unexpected(Error::OUT_OF_MAP);
            case MoveError::BORDER_REACHED:
                return std::unexpected(Error::OUT_OF_MAP);
            case MoveError::INFINITE_LOOP:
                std::cerr << "Reached loop iteration limit in MeshSurfaceModel::move_on_surface!" << std::endl;
                return std::unexpected(Error::PREDICTION_ERROR);
            case MoveError::INTERSECTION_ERROR:
            case MoveError::POSITION_OUTSIDE_ASSOCIATED_TRIANGLE:
            default:
                return std::unexpected(Error::PREDICTION_ERROR);
        }
    }
}


// If the mesh is broken, for example two opposite halfedges do not belong to two
// adjacent faces (which I have experienced), this algorithm failes!
// A way to detect this would be to check if the final position is inside the associated face.
// In this case we would need to fail and probably propagate the error up to the Optimizer,
// so the trajectory can be discarded.
std::expected<SurfacePose, MeshSurfaceModel::MoveError> MeshSurfaceModel::move_on_surface(
    SurfacePose pose,
    Vector direction,
    float distance
)
{
    // Use the pmp mesh directly
    const auto& pmp_mesh = mesh_->getSurfaceMesh();
    lvr2::FaceHandle faceH = pose.face;

    // 1. We need to find out if the new point lies inside or outside the current triangle
    // Actually we do not, the caller has to set the correct face handle!
    // Current Face and Normal
    Normal faceN = normals_[faceH];

    // Direction and distance to go
    Vector dir = mesh_map::projectVectorOntoPlane(
        direction,
        Vector(0, 0, 0),
        faceN
    ).normalized();

    // Project the direction Vector onto the plane along the robots up axis
    {
        // Basically Ray-Plane intersection point calculation
        const Vector up_axis = to_map_vec(pose.pose.orientation * Eigen::Vector3f::UnitZ());
        const float t = (direction * -1.0f).dot(faceN) / (faceN.dot(up_axis));
        dir = direction + up_axis * t;
        dir.normalize();
    }
    // Project the position onto the plane
    {
        const float pdist = signed_point_to_plane_distance(
            pose.pose.position,
            mesh_->getVertexPositionsOfFace(faceH)[0], faceN
        );
        pose.pose.position = pose.pose.position - faceN * pdist;
    }

    float dist_to_go = distance;
    float intersection_dist = std::numeric_limits<float>::infinity();
    uint32_t loop_cnt = 0;
    do
    {
        lvr2::OptionalEdgeHandle closest;

        // Get the triangles vertices in counter clockwise order
        auto tmp = pmp_mesh.halfedges(faceH);
        const std::array half_edges = {*tmp, *(++tmp), *(++tmp)};
        const std::array<Vector, 3> positions = {
            mesh_->getVertexPosition(pmp_mesh.from_vertex(half_edges[0])),
            mesh_->getVertexPosition(pmp_mesh.from_vertex(half_edges[1])),
            mesh_->getVertexPosition(pmp_mesh.from_vertex(half_edges[2]))
        };

        // Check if the end point is inside the current triangle, if so we can skip all this
        const Vector end = pose.pose.position + dir * dist_to_go;
        if (point_inside_triangle(end, positions))
        {
            break;
        }

        // Find the first edge intersected from the inside of the triangle.
        // If the ray does not intersect the triangle this might return no result!
        const auto closest_intersection = intersect_first_outgoing_edge(positions, pose.pose.position, dir, faceN);
        if (!closest_intersection)
        {
            return std::unexpected(MoveError::INTERSECTION_ERROR);
        }

        closest = half_edges[closest_intersection->first].edge();
        intersection_dist = closest_intersection->second;

        // This check is necessary because point_inside_triangle can fail for points on
        // or very close to an edge
        if (dist_to_go < intersection_dist)
        {
            break;
        }

        assert(closest);
        lvr2::EdgeHandle eH = closest.unwrap();

        // Limit the number of loops to a fixed n
        loop_cnt++;
        if (loop_cnt >= 50)
        {
            return std::unexpected(MoveError::INFINITE_LOOP);
        }

        // Move the pos to the border
        pose.pose.position += dir * intersection_dist;
        dist_to_go -= intersection_dist;
        // Update the face
        if (const auto opt = opposite_face(faceH, mesh_->getFacesOfEdge(eH)))
        {
            faceH = opt.value();
        }
        else
        {
            return std::unexpected(MoveError::BORDER_REACHED);
        }

        // Get the new Normal and rotate the direction vector to stay in the plane
        dir = rotate_direction_onto_new_triangle(dir, faceN, normals_[faceH]);
        // Adjust the orientation
        pose.pose.orientation = rotate_orientation_onto_new_triangle(pose.pose.orientation, faceN, normals_[faceH]);
        faceN = normals_[faceH];

        // Reproject the point onto the plane to avoid accumulating drift over time
        pose.pose.position = project_point_to_plane(pose.pose.position, mesh_->getVertexPositionsOfFace(faceH)[0], faceN);
    }
    // Move to the next intersection until we do not have to leave the triangle
    while(true);
    
    // Update the position one last time, orientation is already done each time we switch triangle
    pose.pose.position += dir * dist_to_go;
    // Set the new face
    pose.face = faceH;

    // NOTE: This would return false for points on the edge but the distance is
    // always very small, and because the intersection algorithm above can handle
    // the start point being outside the triangle we just ignore it here.
    //
    // if (!point_inside_triangle(pos, mesh_->getVertexPositionsOfFace(faceH)))
    // {
    //     return std::unexpected(MoveError::POSITION_OUTSIDE_ASSOCIATED_TRIANGLE);
    // }

    return pose;
}


/**
 * @brief Calculate the intersection of a line segment with the X-Axis.
 *
 * This is only used in the intersect_first_outgoing_edge, which is why it has no
 * declaration in the header and is `inline`.
 *
 * @param a First vector of the line segment
 * @þaram b Second vector of the line segment
 * @return Intersection point of the line segment with the X-Axis if it exists.
 */
inline std::optional<float> calc_x_axis_intersection(const Vector& a, const Vector& b)
{
    // Check if parallel
    if (a.y == b.y)
    {
        return std::nullopt;
    }

    // Check if segment crosses the X-Axis.
    // Edge-Case: When one triangle point is on the X-Axis both signs are positive
    // so we cannot simply check if the signs are equal with std::signbit
    if (0.0f < a.y && 0.0f < b.y)
    {
        return std::nullopt;
    }
    if (0.0f > a.y && 0.0f > b.y)
    {
        return std::nullopt;
    }

    // Check if the line is vertical
    if (a.x == b.x)
    {
        return a.x;
    }

    // Calc inclination
    const float m = (b.y - a.y) / (b.x - a.x);

    // X intersection (y = 0)
    return (-a.y / m) + a.x;
}


std::optional<std::pair<int, float>> intersect_first_outgoing_edge(
    const std::array<Vector, 3>& triangle,
    const Vector& origin,
    const Vector& direction,
    const Normal& normal
)
{
    std::array<std::optional<float>, 3> intersections;

    // Transformation from the ray local coordinate system to the map system
    Eigen::Isometry3f m = Eigen::Isometry3f::Identity();
    // (0, 0, 1) x (1, 0, 0) = (0, 1, 0)
    const Vector y = normal.cross(direction);
    // X-Axis
    m(0, 0) = direction.x;
    m(1, 0) = direction.y;
    m(2, 0) = direction.z;
    // Y-Axis
    m(0, 1) = y.x;
    m(1, 1) = y.y;
    m(2, 1) = y.z;
    // Z-Axis
    m(0, 2) = normal.x;
    m(1, 2) = normal.y;
    m(2, 2) = normal.z;
    // Translation
    m(0, 3) = origin.x;
    m(1, 3) = origin.y;
    m(2, 3) = origin.z;

    const Eigen::Isometry3f inv = m.inverse();

    // Transform the triangle to the ray local coordinate system to solve the 2D intersection problem
    const std::array<Vector, 3> local_triangle = {
        inv.matrix() * triangle[0],
        inv.matrix() * triangle[1],
        inv.matrix() * triangle[2]
    };

    // Intersect the edges if the ray is comming from inside the triangle
    // Vertices of triangles are ordered counterclockwise -> edges with positive
    // inclination in the ray-local coordinate system are intersected from
    // the inside of the triangle.
    if (local_triangle[1].y > local_triangle[0].y)
    {
        // Compute intersection with the ray
        intersections[0] = calc_x_axis_intersection(local_triangle[0], local_triangle[1]);
    }

    if (local_triangle[2].y > local_triangle[1].y)
    {
        // Compute intersection with the ray
        intersections[1] = calc_x_axis_intersection(local_triangle[1], local_triangle[2]);
    }

    if (local_triangle[0].y > local_triangle[2].y)
    {
        // Compute intersection with the ray
        intersections[2] = calc_x_axis_intersection(local_triangle[2], local_triangle[0]);
    }

    // Find the closest intersection
    float min_dist = std::numeric_limits<float>::infinity();
    std::optional<int> edge = std::nullopt;

    for (uint32_t i = 0; i < 3; i++)
    {
        if (intersections[i] && intersections[i].value() < min_dist)
        {
            edge = i;
            min_dist = intersections[i].value();
        }
    }

    if (!edge)
    {
        return std::nullopt;
    }

    return std::pair(edge.value(), min_dist);
}

} // namespace mesh_mppi

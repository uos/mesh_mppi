#pragma once

#include <mesh_map/mesh_map.h>
#include <expected>

#include <mesh_mppi/types/Pose.hpp>
#include <mesh_mppi/Error.hpp>
#include <mesh_mppi/Util.hpp>

namespace mesh_mppi
{

class MeshSurfaceModel;

// Interface to make the Model easily exchangeable
using SurfaceModel = MeshSurfaceModel;

class MeshSurfaceModel
{
public:

    using SharedPtr = typename std::shared_ptr<MeshSurfaceModel>;
    using UniquePtr = typename std::unique_ptr<MeshSurfaceModel>;

    /**
     *  @brief Construct the environment model from the mesh map.
     *
     *  Reads the face normals using map->meshIO(), computes them if not found.
     *
     *  @param map the mesh map
     */
    MeshSurfaceModel(const mesh_map::MeshMap::Ptr& map);

    /**
     *  @brief Construct the environment model.
     *
     *  Always calculates th face normals. This constructor is mainly used for tests.
     *
     *  @param mesh The mesh to use
     */
    MeshSurfaceModel(const std::shared_ptr<lvr2::PMPMesh<mesh_map::Vector>>& mesh);
    
    /**
     * @brief Calculates the next on surface pose of the system
     * based on the current pose and a 2D pose delta
     *
     * @param current   The current pose of the system
     * @param delta     The pose delta to apply
     *
     * @return          The new on surface pose
     */
    [[nodiscard]]
    std::expected<SurfacePose, Error> predict(const SurfacePose& current, const Pose2D& delta) noexcept;

    enum class MoveError
    {
        OUT_OF_MAP,
        BORDER_REACHED,
        INFINITE_LOOP,
        INTERSECTION_ERROR,
        // The determined position is outside of the associated triangle
        POSITION_OUTSIDE_ASSOCIATED_TRIANGLE
    };
    /**
     *  @brief Moves a point by the given displacement, ensuring the point stays on the surface of the mesh
     *  @param position The point to move
     *  @param direction The direction of movement
     *  @param distance The distance to move
     *
     *  @return The new position and direction in case no error occurred
     */
    std::expected<SurfacePose, MoveError> move_on_surface(
        SurfacePose pose,
        Vector direction,
        float distance
    );

private:

    inline Vector rotate_direction_onto_new_triangle(const Vector& dir, const Normal& old_normal, const Normal& new_normal) const
    {
        const Vector left = old_normal.cross(dir);
        return left.cross(new_normal);
    }

    inline Eigen::Quaternionf rotate_orientation_onto_new_triangle(const Eigen::Quaternionf& o, const Normal& old_normal, const Normal& new_normal) const
    {
        return Eigen::Quaternionf::FromTwoVectors(to_eigen_vec(old_normal), to_eigen_vec(new_normal)) * o;
    }

    // Map of the environment
    std::shared_ptr<lvr2::PMPMesh<mesh_map::Vector>> mesh_;
    lvr2::DenseFaceMap<Normal> normals_;
};


/**
 * @brief Find the first edge that the ray intersects from inside the triangle.
 *
 * Edges which are intersected when the ray enters the triangle are ignored.
 *
 * @param triangle The vertices of the triangle in counter clockwise order!
 *
 * @return the edge index and distance of the closest outgoing intersection. None if the ray does not cross the triangle.
 */
std::optional<std::pair<int, float>> intersect_first_outgoing_edge(
    const std::array<Vector, 3>& triangle,
    const Vector& origin,
    const Vector& direction,
    const Normal& normal
);

} // namespace mesh_mppi

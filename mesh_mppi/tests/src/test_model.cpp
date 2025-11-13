#include <gtest/gtest.h>
#include <mesh_map/mesh_map.h>

#include <mesh_mppi/Model.hpp>
#include <mesh_mppi/Util.hpp>
#include <mesh_mppi/io/Trajectory.hpp>
#include <mesh_mppi/kinematics/DifferentialDrive.hpp>

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/io/ModelFactory.hpp>
#include <lvr2/geometry/PMPMesh.hpp>

#include <optional>
#include <tf2_ros/transform_listener.h>

#include <mesh_map/util.h>

using namespace mesh_mppi;

// FIXME: Drop the MeshMap dependency in the tests. Use Embree or other BVH for closest point query


mesh_map::MeshMap::Ptr load_map(tf2_ros::Buffer& tf)
{
    // Add parameters for mesh map
    rclcpp::NodeOptions options;
    options.arguments({"--params-file", "./resources/configs/test_model.yaml"});
    options.append_parameter_override("mesh_map.mesh_file", "./resources/meshes/plane.ply");
    options.append_parameter_override("mesh_map.default_layer", "");

    auto node = std::make_shared<rclcpp::Node>("test_model", options);
    auto map = std::make_shared<mesh_map::MeshMap>(tf, node);

    map->readMap();
    return map;
}

TEST(TestModel, move_once)
{
    rclcpp::init(0, nullptr);
    
    auto tf_node = std::make_shared<rclcpp::Node>("tf");
    auto buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    auto mesh_map = load_map(*buffer);

    MeshSurfaceModel model(mesh_map);
    Pose pose;
    pose.position = Vector(0.1, 0.1, 0);
    pose.orientation.setIdentity();
    SurfacePose old(
        pose,
        mesh_map->getContainingFace(pose.position, 0.5).unwrap()
    );
    Pose2D delta;
    delta.x = 0.5;
    delta.y = 0.0;
    delta.theta = 0.0;
    
    auto res = model.predict(old, delta);
    
    if (!res)
    {
        rclcpp::shutdown();
        GTEST_FAIL() << "model.predict returned an Error code! " << int(res.error());
        return;
    }

    SurfacePose s = res.value();
    
    EXPECT_NEAR(s.pose.position.x, 0.6, 1e6) << "Unexpected x position of new pose";
    EXPECT_NEAR(s.pose.position.y, 0.1, 1e6) << "Unexpected y position of new pose";
    EXPECT_NEAR(s.pose.position.z, 0.0, 1e6) << "Unexpected z position of new pose";
    EXPECT_EQ(s.pose.orientation, Quaternion::Identity()) << "Unexpected orientation of new pose";
    rclcpp::shutdown();
}

// TODO: Add test for edge case origin is incident with vertex
// TODO: Add test for edge case ray is parallel to edge


TEST(TestModel, move_in_square)
{
    rclcpp::init(0, nullptr);
    
    auto tf_node = std::make_shared<rclcpp::Node>("tf");
    auto buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    auto mesh_map = load_map(*buffer);

    MeshSurfaceModel model(mesh_map);
    Pose pose;
    pose.position = Vector(0.01, 0.01, 0);
    pose.orientation.setIdentity();
    SurfacePose old(
        pose,
        mesh_map->getContainingFace(pose.position, 0.5).unwrap()
    );
    Pose2D delta;
    delta.x = 0.5;
    delta.y = 0.0;
    delta.theta = 0.0;
        
    SurfacePose s = old;
    for (int i = 0; i < 4; i++)
    {
        if (auto res = model.predict(s, delta))
        {
            s = res.value();
        }
        else
        {
            rclcpp::shutdown();
            GTEST_FAIL() << "model.predict returned an Error code! " << int(res.error());
            return;
        }
    }
    EXPECT_LE(s.pose.position.distance(old.pose.position), 1e-6) << "Pose delta of new pose too large";
    EXPECT_LE(s.pose.orientation.angularDistance(old.pose.orientation), 1e-6) << "Orientation Delta of new pose too large";
    EXPECT_EQ(s.face.idx(), old.face.idx()) << "Unexpected face of new pose";
    rclcpp::shutdown();
}


TEST(TestModel, in_place_rotation)
{
    rclcpp::init(0, nullptr);
    
    auto tf_node = std::make_shared<rclcpp::Node>("tf");
    auto buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    auto mesh_map = load_map(*buffer);

    MeshSurfaceModel model(mesh_map);
    Pose pose;
    pose.position = Vector(0.01, 0.01, 0);
    pose.orientation.setIdentity();
    SurfacePose old(
        pose,
        mesh_map->getContainingFace(pose.position, 0.5).unwrap()
    );
    Pose2D delta;
    delta.theta = M_PI_2;
        
    SurfacePose s = old;
    for (int i = 0; i < 4; i++)
    {
        if (auto res = model.predict(s, delta))
        {
            s = res.value();
        }
        else
        {
            GTEST_FAIL() << "model.predict returned an Error code!";
        }
    }
    EXPECT_EQ(s.pose.position, old.pose.position) << "The pose should not move";
    EXPECT_LE(s.pose.orientation.angularDistance(old.pose.orientation), 1e-6) << "Orientation Delta of new pose too large";
    EXPECT_EQ(s.face.idx(), old.face.idx()) << "Unexpected face of new pose";
}


// This only works correctly if the pose is actually on a face!
// Otherwise we would need to check if the projected point is inside the triangle
// which failes if the pose is on one of the vertices :(
lvr2::FaceHandle dumb_find_closest_triangle(const std::shared_ptr<lvr2::PMPMesh<Vector>>& mesh, const mesh_mppi::Pose& pose)
{
    float min_dist = std::numeric_limits<float>::infinity();
    lvr2::FaceHandle closest(0);
    for (const lvr2::FaceHandle f: mesh->faces())
    {
        const auto corners = mesh->getVertexPositionsOfFace(f);
        const auto normal = lvr2::getFaceNormal(corners).value();
        const float p2p_dist = mesh_mppi::signed_point_to_plane_distance(
            pose.position,
            corners[0],
            normal
        );

        if (std::abs(p2p_dist) < min_dist)
        {
            min_dist = std::abs(p2p_dist);
            closest = f;
        }
    }

    return closest;
}


TEST(TestModel, move_over_sphere)
{
    lvr2::ModelFactory fac;
    auto buffer = fac.readModel("./resources/meshes/sphere.ply");
    if (!buffer->m_mesh)
    {
        GTEST_SKIP() << "Missing resource 'sphere.ply'";
    }

    auto hem = std::make_shared<lvr2::PMPMesh<Vector>>(buffer->m_mesh);
    mesh_mppi::MeshSurfaceModel model(hem);

    mesh_mppi::SurfacePose initial(mesh_mppi::Pose(), lvr2::FaceHandle(0));
    initial.pose.position.z = 2.0; // The sphere has radius 2
    initial.face = dumb_find_closest_triangle(hem, initial.pose);

    // Set the position to the middle of the face
    initial.pose.position = mesh_map::linearCombineBarycentricCoords(hem->getVertexPositionsOfFace(initial.face), {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0});
    
    Pose2D delta;
    delta.x = 0.1;
    delta.theta = 0.3;

    SurfacePose current = initial;
    for (int i = 0; i < 100; i++)
    {
        // Predict one step
        SurfacePose prev = current;
        if (auto res = model.predict(current, delta))
        {
            current = res.value();
        }
        else
        {
            GTEST_FAIL() << "model.predict returned an Error code!";
        }

        // Ensure the position is on the surface of the face
        const auto corners = hem->getVertexPositionsOfFace(current.face);
        const auto normal = lvr2::getFaceNormal(corners).value();
        EXPECT_LE(
            mesh_mppi::point_to_plane_distance(current.pose.position, corners[0], normal),
            0.001
        ) << "Position is not on the plane implied by the face its associated to!";

        // Ensure the position is inside the associated triangle
        EXPECT_TRUE(mesh_mppi::point_inside_triangle(current.pose.position, corners[0], corners[1], corners[2]));

        // Ensure we actually moved
        EXPECT_LE(0.001, current.pose.position.distanceFrom(prev.pose.position)) << "Index: " << i << "; Pose has not been moved over surface!";
    }
}


TEST(TestIntersectEdges, direction_incident_with_edge)
{
    using namespace mesh_mppi;

    std::array<Vector, 3> triangle = {
        Vector(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0)
    };
    Normal n(0.0, 0.0, 1.0);
    Vector origin(0.0, 0.0, 0.0);
    Vector ray(1.0, 0.0, 0.0);


    // Expect that we get a result and that that result is correct
    EXPECT_EQ(intersect_first_outgoing_edge(triangle, origin, ray, n), std::optional(std::make_pair(1, 1.0f)));
}

TEST(TestIntersectEdges, direction_not_intersecting_triangle)
{
    using namespace mesh_mppi;

    std::array<Vector, 3> triangle = {
        Vector(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0)
    };
    Normal n(0.0, 0.0, 1.0);
    Vector origin(0.0, -1.0, 0.0);
    Vector ray(1.0, 0.0, 0.0);


    // Expect that we do not get a result
    EXPECT_EQ(intersect_first_outgoing_edge(triangle, origin, ray, n), std::nullopt);
}

TEST(TestIntersectEdges, origin_outside_triangle)
{
    using namespace mesh_mppi;

    std::array<Vector, 3> triangle = {
        Vector(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(1.0, 1.0, 0.0)
    };
    Normal n(0.0, 0.0, 1.0);
    Vector origin(0.0, 0.5, 0.0);
    Vector ray(1.0, 0.0, 0.0);


    // Expect that we intersect the furthest edge
    EXPECT_EQ(intersect_first_outgoing_edge(triangle, origin, ray, n), std::optional(std::make_pair(1, 1.0f)));
}

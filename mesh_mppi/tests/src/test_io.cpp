#include <gtest/gtest.h>

#include <mesh_mppi/io/State.hpp>
#include <mesh_mppi/io/Trajectory.hpp>

#include <filesystem>
using std::filesystem::path;
using namespace mesh_mppi;

TEST(TestIO, state)
{
    const path file = "/tmp/mpc/test/state.binary";
    if (std::filesystem::create_directories(file.parent_path()))
    {
        GTEST_FAIL() << "Could not create tmp dir";
    }
    
    std::ofstream outf(file.string(), std::ios::binary);
    State out;
    out.face = lvr2::FaceHandle(1);
    out.pose.position.x = 2;
    out.pose.position.y = 3;
    out.pose.position.z = 4;
    out.pose.orientation.x() = 5;
    out.pose.orientation.y() = 6;
    out.pose.orientation.z() = 7;
    out.pose.orientation.w() = 8;
    out.state = Eigen::VectorXf::Random(3);
    mesh_mppi::io::write(outf, out);
    outf.close();

    std::ifstream inf(file.string(), std::ios::binary);
    State in;

    mesh_mppi::io::read(inf, in);

    ASSERT_EQ(out.face.unwrap().idx(), in.face.unwrap().idx());
    ASSERT_EQ(out.pose.position, in.pose.position);
    ASSERT_EQ(out.pose.orientation, in.pose.orientation);
    ASSERT_EQ(out.state, in.state);
}

TEST(TestIO, trajectory)
{
    const path file = "/tmp/mpc/test/trajectory.binary";
    if (std::filesystem::create_directories(file.parent_path()))
    {
        GTEST_FAIL() << "Could not create tmp dir";
    }

    State state;
    state.face = lvr2::FaceHandle(1);
    state.pose.position.x = 2;
    state.pose.position.y = 3;
    state.pose.position.z = 4;
    state.pose.orientation.x() = 5;
    state.pose.orientation.y() = 6;
    state.pose.orientation.z() = 7;
    state.pose.orientation.w() = 8;
    state.state = Eigen::VectorXf::Random(3);
    Trajectory out(50, state);

    std::ofstream outf(file.string(), std::ios::binary);
    mesh_mppi::io::write(outf, out);
    outf.close();

    std::ifstream inf(file.string(), std::ios::binary);
    Trajectory in;
    mesh_mppi::io::read(inf, in);
    
    ASSERT_EQ(out, in);
}

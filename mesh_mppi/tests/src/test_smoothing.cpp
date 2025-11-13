#include <gtest/gtest.h>

#include <mesh_mppi/Smoother.hpp>

TEST(TestSmoother, smooth_one_signal)
{
    constexpr const int N_CONTROLS = 1;
    Smoother<N_CONTROLS> sm;

    Eigen::VectorXf seq(30 * N_CONTROLS);
    seq.setOnes();

    Eigen::Array4f history = {1.0, 1.0, 1.0, 1.0};

    Eigen::ArrayXf res = sm.smooth_one_signal(history, seq);

    // Expected values were generated with scipy
    Eigen::ArrayXf expected(30);
    expected <<
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.09090909, 1.03030303, 0.86147186, 0.62770563;

    EXPECT_EQ(res.rows(), expected.rows());
    for (Eigen::Index i = 0; i < expected.rows(); i++)
    {
        EXPECT_FLOAT_EQ(res(i), expected(i));
    }
}

TEST(TestSmoother, smooth)
{
    constexpr const int N_CONTROLS = 2;
    Smoother<N_CONTROLS> sm;

    Eigen::VectorXf seq(30 * N_CONTROLS);
    // Set the data
    auto view = seq.reshaped<Eigen::RowMajor>(30, N_CONTROLS);
    view.col(0).setZero();
    view.col(1).setOnes();

    Eigen::Array<float, 4, N_CONTROLS> history;
    history.col(0).setOnes();
    history.col(1).setZero();

    Eigen::ArrayXf res = sm.smooth(history, seq);

    // Column 0
    Eigen::ArrayXf expected(30);
    expected << 0.37229437, 0.13852814, -0.03030303, -0.090909091, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;



    EXPECT_EQ(res.rows(), seq.rows());
    for (Eigen::Index i = 0; i < expected.rows(); i++)
    {
        EXPECT_FLOAT_EQ(res(i * N_CONTROLS + 0), expected(i));
    }


    // Column 1
    expected << 0.62770563, 0.86147186, 1.030303, 1.0909091, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0909091, 1.030303, 0.86147186, 0.62770563;

    for (Eigen::Index i = 0; i < expected.rows(); i++)
    {
        EXPECT_FLOAT_EQ(res(i * N_CONTROLS + 1), expected(i)) << "Values differ at row " << i;
    }
}

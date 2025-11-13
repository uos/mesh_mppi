#pragma once

#include <Eigen/Dense>
#include <cstdlib>


template <int NControls>
class Smoother
{
    // Savitzky-Golay smoothing kernel
    Eigen::Array<float, 9, 1> kernel_;

public:

    Smoother()
    : kernel_({-21, 14, 39, 54, 59, 54, 39, 14, -21})
    {
        kernel_ /= 231.0f;
    }
    
    // Laplacian smooth a vector of control signals (One trajectory)
    Eigen::VectorXf smooth(const Eigen::Array<float, 4, NControls>& history, const Eigen::VectorXf& controls) const
    {
        auto control = controls.reshaped<Eigen::RowMajor>(controls.rows() / NControls, NControls);
        Eigen::VectorXf result(controls.rows());
        auto result_view = result.reshaped<Eigen::RowMajor>(controls.rows() / NControls, NControls);

        for (Eigen::Index col = 0; col < NControls; col++)
        {
            result_view.col(col) = smooth_one_signal(history.col(col), control.col(col));
        }

        return result;
    }

    Eigen::ArrayXf smooth_one_signal(const Eigen::Array4f& history, const Eigen::ArrayXf& sequence) const
    {
        Eigen::Array<float, 9, 1> data;
        Eigen::ArrayXf res(sequence.rows());
        for (Eigen::Index i = 0; i < sequence.rows(); i++)
        {
            // Copy from i - 4 to i + 4
            for (Eigen::Index j = 0; j < 9; j++)
            {
                Eigen::Index di = i - 4 + j;
                if (di < 0)
                {
                    data(j) = history(4 + di);
                }
                else if (di >= sequence.rows())
                {
                    data(j) = sequence(sequence.rows() - 1);
                }
                else
                {
                    data(j) = sequence(di);
                }
            }

            res(i) = (data * kernel_).sum();
        }

        return res;
    }
};

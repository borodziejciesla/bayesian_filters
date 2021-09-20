/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_IO_FILTER_CALIBRATION_HPP_
#define INCLUDE_IO_FILTER_CALIBRATION_HPP_

#include <functional>

#include <Eigen/Dense>

namespace bf_io
{
    struct FilterCalibration
    {
        size_t state_dimension_ = 0u;
        size_t measurement_dimension_ = 0u;

        std::function<Eigen::VectorXf(const Eigen::VectorXf & state, const float time_delta)> transition;
        std::function<Eigen::MatrixXf(const Eigen::VectorXf & state)> transition_jacobian;

        std::function<Eigen::VectorXf(const Eigen::VectorXf & state)> observation;
        std::function<Eigen::MatrixXf(const Eigen::VectorXf & state)> observation_jacobian;

        Eigen::MatrixXf proccess_noise_covariance;
    };
}   // namespace bf_io

#endif  //  INCLUDE_IO_FILTER_CALIBRATION_HPP_

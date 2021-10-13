/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "extended_kalman_filter.hpp"

namespace bf
{
    ExtendedKalmanFilter::ExtendedKalmanFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration) {
    }

    ExtendedKalmanFilter::StateWithCovariance ExtendedKalmanFilter::Prediction(const float time_delta) {
        auto predicted_state = transition_(estimated_state_, time_delta);
        auto jacobian = transition_jacobian_(estimated_state_, time_delta);

        auto predicted_covariance = jacobian * estimated_covariance_ * jacobian.transpose() + process_noise_covariance_;

        return std::make_tuple(predicted_state, predicted_covariance);
    }

    ExtendedKalmanFilter::StateWithCovariance ExtendedKalmanFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
        const Eigen::VectorXf & predicted_state,
        const Eigen::MatrixXf & predicted_covariance) {
        auto [y, R] = ConvertMeasurement(measurement);
        Eigen::MatrixXf observation_jacobian = observation_jacobian_(predicted_state);

        Eigen::MatrixXf predicted_measurement = observation_(predicted_state);
        Eigen::MatrixXf innovation = y - predicted_measurement;
        Eigen::MatrixXf innovation_covariance = observation_jacobian * predicted_covariance * observation_jacobian.transpose() + R;
        Eigen::MatrixXf kalman_gain = predicted_covariance * observation_jacobian.transpose() * innovation_covariance.inverse();

        Eigen::MatrixXf estimated_state = static_cast<Eigen::MatrixXf>(predicted_state + kalman_gain * innovation);
        Eigen::MatrixXf tmp = static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Identity(predicted_state.size(), predicted_state.size())) - kalman_gain * observation_jacobian;
        Eigen::MatrixXf estimated_covariance = static_cast<Eigen::MatrixXf>(tmp * predicted_covariance * tmp.transpose())
            + static_cast<Eigen::MatrixXf>(kalman_gain * R * kalman_gain.transpose());

        return std::make_tuple(estimated_state, estimated_covariance);
    }
}   // namespace bf

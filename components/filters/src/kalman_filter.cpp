/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "kalman_filter.hpp"

namespace bf
{
    KalmanFilter::KalmanFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration) {
    }

    void KalmanFilter::Prediction(const float time_delta) {
        predicted_state_ = transition_(estimated_state_, std::nullopt, time_delta);
        auto jacobian = transition_jacobian_(estimated_state_, time_delta);
        predicted_covariance_ = jacobian * estimated_covariance_ * jacobian.transpose();
    }

    void KalmanFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        auto [y, R] = ConvertMeasurement(measurement);
        Eigen::MatrixXf observation_jacobian = observation_jacobian_(predicted_state_);

        Eigen::MatrixXf predicted_observation = observation_(predicted_state_);
        Eigen::MatrixXf innovation = y - predicted_observation;
        Eigen::MatrixXf innovation_covariance = observation_jacobian * predicted_covariance_ * observation_jacobian.transpose() + R;
        Eigen::MatrixXf kalman_gain = predicted_covariance_ * observation_jacobian.transpose() * innovation_covariance.inverse();

        estimated_state_ = static_cast<Eigen::MatrixXf>(predicted_state_ + kalman_gain * innovation);
        Eigen::MatrixXf tmp = static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Identity(predicted_state_.size(), predicted_state_.size()) - kalman_gain * observation_jacobian);

        estimated_covariance_ = static_cast<Eigen::MatrixXf>( static_cast<Eigen::MatrixXf>(tmp * predicted_covariance_ * tmp.transpose()))
            + static_cast<Eigen::MatrixXf>(static_cast<Eigen::MatrixXf>(kalman_gain * R * kalman_gain.transpose()));
    }
}   // namespace bf

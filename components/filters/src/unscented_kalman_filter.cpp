/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "unscented_kalman_filter.hpp"

#include <algorithm>
#include <numeric>

#include <Eigen/Cholesky>

namespace bf
{
    UnscentedKalmanFilter::UnscentedKalmanFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration) {
        sigma_points_number_ = 2u * dimension_ + 1u;

        sigma_points_.resize(sigma_points_number_);

        sigma_points_.at(0) = std::make_pair(Eigen::VectorXf::Zero(dimension_),
            lambda_ / (static_cast<float>(dimension_) + lambda_));
        std::transform(sigma_points_.begin() + 1u, sigma_points_.end(),
            sigma_points_.begin() + 1u,
            [this](auto arg) {
                std::ignore = arg;
                return std::make_pair(Eigen::VectorXf::Zero(dimension_),
                    (1.0f - sigma_points_.at(0).second) / (2.0f * static_cast<float>(dimension_)) );
            }
        );
    }

    void UnscentedKalmanFilter::Prediction(const float time_delta) {
        FindSigmaPoints();
        PredictSigmaPoints(time_delta);
        PredictMeanAndCovariance();
    }

    void UnscentedKalmanFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        auto converted_measurement = ConvertMeasurement(measurement);

        auto predicted_measurement = PredictMeasurement(converted_measurement);
        UpdateState(predicted_measurement, converted_measurement);
    }

    void UnscentedKalmanFilter::FindSigmaPoints(void)
    {
        Eigen::MatrixXf S = (static_cast<float>(dimension_) + lambda_) * estimated_covariance_;
        Eigen::MatrixXf direction = S.llt().matrixL();

        /* Central */
        sigma_points_.at(0).first = estimated_state_;

        /* Plus/Mius Sigma points */
        auto idx = 0u;
        std::for_each(sigma_points_.begin() + 1u, sigma_points_.begin() + dimension_ + 1u,
            [this,direction,&idx](SigmaPointWithWeight & sigma_point) {
                sigma_point.first = estimated_state_ + static_cast<Eigen::VectorXf>(direction.col(idx++));
            }
        );

        idx = 0u;
        std::for_each(sigma_points_.begin() + dimension_ + 1u, sigma_points_.end(),
            [this,direction,&idx](SigmaPointWithWeight & sigma_point) {
                sigma_point.first = estimated_state_ - static_cast<Eigen::VectorXf>(direction.col(idx++));
            }
        );
    }

    void UnscentedKalmanFilter::PredictSigmaPoints(const float time_delta) {
        std::transform(sigma_points_.begin(), sigma_points_.end(),
            sigma_points_.begin(),
            [=,this](SigmaPointWithWeight & sigma_point) {
                return std::make_pair(transition_(sigma_point.first, time_delta), sigma_point.second);
            }
        );
    }

    void UnscentedKalmanFilter::PredictMeanAndCovariance(void) {
        /* Predicted Value */
        predicted_state_ = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::VectorXf>(Eigen::VectorXf::Zero(dimension_)),
            [](Eigen::VectorXf accumulated, const SigmaPointWithWeight & sigma_point) {
                return accumulated + sigma_point.first * sigma_point.second;
            }
        );

        /* Predicted Covariance */
        predicted_covariance_ = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(dimension_, dimension_)),
            [this](Eigen::MatrixXf accumulated, const SigmaPointWithWeight & sigma_point) {
                auto difference = sigma_point.first - predicted_state_;
                return accumulated + sigma_point.second * (difference * difference.transpose());
            }
        ) + process_noise_covariance_;
    }

    UnscentedKalmanFilter::ValueAndCovariance UnscentedKalmanFilter::PredictMeasurement(const StateWithCovariance & measurement) {
        /* Predicted Measurement */
        Eigen::MatrixXf predicted_measurement = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::VectorXf>(Eigen::VectorXf::Zero(measurement_dimension_)),
            [this](Eigen::VectorXf accumulated, const SigmaPointWithWeight & sigma_point) {
                auto observation = observation_(sigma_point.first);
                auto weighted_observation = static_cast<Eigen::VectorXf>(observation * sigma_point.second);
                auto sum = static_cast<Eigen::VectorXf>(accumulated + weighted_observation);
                return sum;
            }
        );

        /* Predicted Covariance */
        Eigen::MatrixXf predicted_covariance = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(measurement_dimension_, measurement_dimension_)),
            [this,predicted_measurement](Eigen::MatrixXf accumulated, const SigmaPointWithWeight & sigma_point) {
                auto difference = static_cast<Eigen::VectorXf>(observation_(sigma_point.first) - predicted_measurement);
                return static_cast<Eigen::MatrixXf>(accumulated + sigma_point.second * (difference * difference.transpose()));
            }
        ) + std::get<1>(measurement);

        return std::make_pair(predicted_measurement, predicted_covariance);
    }

    void UnscentedKalmanFilter::UpdateState(const ValueAndCovariance & predicted_measurement, const StateWithCovariance & measurement) {
        /* Calculate T */
        auto T = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(dimension_, measurement_dimension_)),
            [this,predicted_measurement](Eigen::MatrixXf accumulated, const SigmaPointWithWeight & sigma_point) {
                auto diff_state = static_cast<Eigen::MatrixXf>(sigma_point.first - predicted_state_);
                auto diff_measurement = static_cast<Eigen::MatrixXf>(observation_(sigma_point.first) - predicted_measurement.first);
                auto cross_single = static_cast<Eigen::MatrixXf>(diff_state * diff_measurement.transpose());
                return static_cast<Eigen::MatrixXf>(accumulated + sigma_point.second * cross_single);
            }
        );
        
        /* Kalman Gain */
        auto kalman_gain = static_cast<Eigen::MatrixXf>(T * predicted_measurement.second.inverse());

        /* Calculate estimate */
        auto innovation = static_cast<Eigen::VectorXf>(std::get<0>(measurement) - predicted_measurement.first);
        estimated_state_ = static_cast<Eigen::VectorXf>(static_cast<Eigen::VectorXf>(predicted_state_) + static_cast<Eigen::VectorXf>(kalman_gain * innovation));
        estimated_covariance_ = static_cast<Eigen::MatrixXf>(predicted_covariance_ - static_cast<Eigen::MatrixXf>(kalman_gain * predicted_measurement.second * kalman_gain.transpose()));
    }
}   // namespace bf

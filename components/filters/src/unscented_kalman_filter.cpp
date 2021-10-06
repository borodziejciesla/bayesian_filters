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
                    lambda_ / (2.0f * (static_cast<float>(dimension_) + lambda_)) );
            }
        );
    }

    CoreBayesianFilter::StateWithCovariance UnscentedKalmanFilter::Prediction(const float time_delta) {
        FindSigmaPoints();
        PredictSigmaPoints(time_delta);
        PredictMeanAndCovariance();

        return std::make_tuple(predicted_state_, predicted_covariance_);
    }

    CoreBayesianFilter::StateWithCovariance UnscentedKalmanFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
        const Eigen::VectorXf & predicted_state,
        const Eigen::MatrixXf & predicted_covariance) {
        std::ignore = predicted_state;
        std::ignore = predicted_covariance;

        auto converted_measurement = ConvertMeasurement(measurement);

        auto predicted_measurement = PredictMeasurement(converted_measurement);
        return UpdateState(predicted_measurement, converted_measurement);
    }

    void UnscentedKalmanFilter::FindSigmaPoints(void)
    {
        Eigen::MatrixXf S = (static_cast<float>(dimension_) + lambda_) * estimated_covariance_;
        Eigen::MatrixXf direction = S.llt().matrixL();

        /* Central */
        sigma_points_.at(0).first = estimated_state_;

        /* Plus/Mius Sigma points */
        std::for_each(sigma_points_.begin() + 1u, sigma_points_.begin() + dimension_ + 1u,
            [this,direction,idx=0](SigmaPointWithWeight & sigma_point) {
                for (auto row = 0u; row < dimension_; row++)
                    sigma_point.first(row) = estimated_state_(row) + direction(row, idx);
            }
        );

        std::for_each(sigma_points_.begin() + dimension_ + 1u, sigma_points_.end(),
            [this,direction,idx=0](SigmaPointWithWeight & sigma_point) {
                for (auto row = 0u; row < dimension_; row++)
                    sigma_point.first(row) = estimated_state_(row) - direction(row, idx);
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
        );
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
                return static_cast<Eigen::VectorXf>(accumulated + sigma_point.second * (difference * difference.transpose()));
            }
        ) + std::get<1>(measurement);

        return std::make_pair(predicted_measurement, predicted_covariance);
    }

    CoreBayesianFilter::StateWithCovariance UnscentedKalmanFilter::UpdateState(const ValueAndCovariance & predicted_measurement,
        const StateWithCovariance & measurement) {
        /* Calculate T */
        auto T = std::accumulate(sigma_points_.begin(), sigma_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(dimension_, measurement_dimension_)),
            [this,predicted_measurement](Eigen::MatrixXf accumulated, const SigmaPointWithWeight & sigma_point) {
                auto diff_state = sigma_point.first - predicted_state_;
                auto diff_measurement = observation_(sigma_point.first) - predicted_measurement.first;
                return accumulated + diff_state * diff_measurement.transpose();
            }
        );
        
        /* Kalman Gain */
        auto kalman_gain = T * predicted_measurement.second.transpose();

        /* Calculate estimate */
        auto estimated_state = predicted_measurement.first + kalman_gain * (std::get<0>(measurement) - predicted_measurement.first);
        auto estimated_covariance = (Eigen::MatrixXf::Identity(dimension_, dimension_) - kalman_gain * T) * predicted_covariance_;

        return std::make_tuple(estimated_state, estimated_covariance);
    }
}   // namespace bf

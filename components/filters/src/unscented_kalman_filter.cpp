/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "unscented_kalman_filter.hpp"

namespace bf
{
    UnscentedKalmanFilter::UnscentedKalmanFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration) {
        dimension_ = calibration.state_dimension_;

        central_weight_ = lambda_ / (static_cast<float>(dimension_) + lambda_);
        central_sigma_point_ = Eigen::VectorXf::Zero(dimension_);

        plus_sigma_points_.resize(dimension_);
        plus_sigma_points_.assign(dimension_, Eigen::VectorXf::Zero(dimension_));
        plus_sigma_weights_.resize(dimension_);
        plus_sigma_weights_.assign(dimension_, 0.5f / (static_cast<float>(dimension_) + lambda_));

        minus_sigma_points_.resize(dimension_);
        minus_sigma_points_.assign(dimension_, Eigen::VectorXf::Zero(dimension_));
        minus_sigma_weights_.resize(dimension_);
        minus_sigma_weights_.assign(dimension_, 0.5f / (static_cast<float>(dimension_) + lambda_));
    }

    UnscentedKalmanFilter::StateWithCovariance UnscentedKalmanFilter::Prediction(const float time_delta) {
        FindSigmaPoints();
        PredictSigmaPoints();
        PredictMeanAndCovariance();
    }

    UnscentedKalmanFilter::StateWithCovariance UnscentedKalmanFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
        PredictMeasurement();
        UpdateState();
    }

    void UnscentedKalmanFilter::FindSigmaPoints(void)
    {
        auto S = (static_cast<float>(dimension_) + lambda_) * estimated_covariance_;
        auto direction = S.sqrt();

        /* Central */
        central_sigma_point_ = estimated_state_;

        /* Plus/Mius Sigma points */
        for (auto i = 0u; i < dimension_; i++)
        {
            plus_sigma_points_ = central_sigma_point_ + direction;
            minus_sigma_points_ = central_sigma_point_ - direction;
        }
    }

    void UnscentedKalmanFilter::PredictSigmaPoints(void) {
        //
    }

    void UnscentedKalmanFilter::PredictMeanAndCovariance(void) {
        //
    }

    void UnscentedKalmanFilter::PredictMeasurement(void) {
        //
    }

    void UnscentedKalmanFilter::UpdateState(void) {
        //
    }
}   // namespace bf

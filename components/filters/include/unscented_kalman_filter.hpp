/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_INCLUDE_UNSCENTED_KALMAN_FILTER_HPP_
#define COMPONENTS_FILTERS_INCLUDE_UNSCENTED_KALMAN_FILTER_HPP_

#include <vector>

#include "core_bayesian_filter.hpp"
#include "filter_calibration.hpp"
#include "value_with_timestamp_and_covariance.hpp"

namespace bf
{
    class UnscentedKalmanFilter : public CoreBayesianFilter
    {
        public:
            UnscentedKalmanFilter(void) = delete;
            UnscentedKalmanFilter(const UnscentedKalmanFilter & obj) = delete;
            UnscentedKalmanFilter(UnscentedKalmanFilter && obj) = delete;
            ~UnscentedKalmanFilter(void) = default;

            explicit UnscentedKalmanFilter(const bf_io::FilterCalibration & calibration);

        protected:
            StateWithCovariance Prediction(const float time_delta);
            StateWithCovariance Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
                const Eigen::VectorXf & predicted_state,
                const Eigen::MatrixXf & predicted_covariance);

        private:
            void FindSigmaPoints(void);
            void PredictSigmaPoints(void);
            void PredictMeanAndCovariance(void);

            void PredictMeasurement(void);
            void UpdateState(void);

            size_t dimension_ = 0u;
            float lambda_ = 3.0f;

            float central_weight_;
            Eigen::VectorXf central_sigma_point_;
            std::vector<float> plus_sigma_weights_;
            std::vector<Eigen::VectorXf> plus_sigma_points_;
            std::vector<float> minus_sigma_weights_;
            std::vector<Eigen::VectorXf> minus_sigma_points_;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_INCLUDE_UNSCENTED_KALMAN_FILTER_HPP_

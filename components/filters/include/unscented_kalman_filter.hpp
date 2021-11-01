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
            void Prediction(const float time_delta) override;
            void Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) override;

        private:
            void FindSigmaPoints(void);
            void PredictSigmaPoints(const float time_delta);
            void PredictMeanAndCovariance(void);

            using ValueAndCovariance = std::pair<Eigen::VectorXf, Eigen::MatrixXf>;

            ValueAndCovariance PredictMeasurement(const StateWithCovariance & measurement);
            void UpdateState(const ValueAndCovariance & predicted_measurement, const StateWithCovariance & measurement);
            
            size_t sigma_points_number_ = 0u;
            float lambda_ = 3.0f;

            using SigmaPointWithWeight = std::pair<Eigen::VectorXf, float>;

            std::vector<SigmaPointWithWeight> sigma_points_;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_INCLUDE_UNSCENTED_KALMAN_FILTER_HPP_

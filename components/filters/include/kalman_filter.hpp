/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_INCLUDE_KALMAN_FILTER_HPP_
#define COMPONENTS_FILTERS_INCLUDE_KALMAN_FILTER_HPP_

#include "core_bayesian_filter.hpp"
#include "filter_calibration.hpp"
#include "value_with_timestamp_and_covariance.hpp"

namespace bf
{
    class KalmanFilter : public CoreBayesianFilter
    {
        public:
            KalmanFilter(void) = delete;
            KalmanFilter(const KalmanFilter & obj) = delete;
            KalmanFilter(KalmanFilter && obj) = delete;
            ~KalmanFilter(void) = default;

            explicit KalmanFilter(const bf_io::FilterCalibration & calibration);

        protected:
            void Prediction(const float time_delta) override;
            void Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) override;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_INCLUDE_KALMAN_FILTER_HPP_

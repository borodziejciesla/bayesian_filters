/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_
#define COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_

#include "filter_calibration.hpp"
#include "value_with_timestamp_and_covariance.hpp"

namespace bf
{
    class CoreBayesianFilter
    {
        public:
            CoreBayesianFilter(void) = delete;
            CoreBayesianFilter(const CoreBayesianFilter & obj) = delete;
            CoreBayesianFilter(CoreBayesianFilter && obj) = delete;
            ~CoreBayesianFilter(void) = default;

            explicit CoreBayesianFilter(const bf_io::FilterCalibration & calibration);

            void RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement);
            const bf_io::ValueWithTimestampAndCovariance & GetEstimation(void);

        protected:
            virtual void RunFilterInternal(const bf_io::ValueWithTimestampAndCovariance & measurement) = 0;

            bf_io::ValueWithTimestampAndCovariance state_;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_

/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_BAYESIAN_FILTER_HPP_
#define INCLUDE_BAYESIAN_FILTER_HPP_

#include <memory>

#include "filter_calibration.hpp"
#include "filter_type.hpp"
#include "value_with_timestamp_and_covariance.hpp"

namespace bf
{
    class CoreBayesianFilter;

    class BayesianFilter
    {
        public:
            BayesianFilter(void) = delete;
            BayesianFilter(const BayesianFilter & obj) = delete;
            BayesianFilter(BayesianFilter && obj) = delete;
            ~BayesianFilter(void);

            BayesianFilter(const bf_io::FilterType filter_type, const bf_io::FilterCalibration & calibration);

            void RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement);
            const bf_io::ValueWithTimestampAndCovariance & GetEstimation(void);

        private:
            std::unique_ptr<CoreBayesianFilter> core_filter_;
            bf_io::FilterCalibration calibration_;
    };
}   // namespace bf

#endif  //  INCLUDE_BAYESIAN_FILTER_HPP_

/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "bayesian_filter.hpp"

#include "bayesian_filter_factory.hpp"

namespace bf
{
    BayesianFilter::BayesianFilter(const bf_io::FilterType filter_type, const bf_io::FilterCalibration & calibration)
        : calibration_{calibration} {
        core_filter_ = BayesianFilterFactory::CreateFilter(filter_type, calibration_);
    }

    void BayesianFilter::RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        core_filter_->RunFilter(measurement);
    }

    const bf_io::ValueWithTimestampAndCovariance & BayesianFilter::GetEstimation(void) {
        return core_filter_->GetEstimation();
    }
}   // namespace bf

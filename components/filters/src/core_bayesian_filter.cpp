/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "core_bayesian_filter.hpp"

namespace bf
{
    CoreBayesianFilter::CoreBayesianFilter(const bf_io::FilterCalibration & calibration) {
    }

    void CoreBayesianFilter::RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        RunFilterInternal(measurement);
    }
    
    const bf_io::ValueWithTimestampAndCovariance & CoreBayesianFilter::GetEstimation(void) {
        return state_;
    }
}   // namespace bf

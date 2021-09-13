/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "kalman_filter.hpp"

namespace bf
{
    KalmanFilter::KalmanFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration) {
    }

    void KalmanFilter::RunFilterInternal(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        
    }
}   // namespace bf

/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "bayesian_filter_factory.hpp"

#include <stdexcept>

#include "extended_kalman_filter.hpp"
#include "kalman_filter.hpp"
#include "particle_filter.hpp"
#include "unscented_kalman_filter.hpp"

namespace bf
{
    std::unique_ptr<CoreBayesianFilter> BayesianFilterFactory::CreateFilter(const bf_io::FilterType filter_type,
        const bf_io::FilterCalibration & calibration) {
        switch (filter_type) {
            case bf_io::FilterType::KF:
                return std::make_unique<KalmanFilter>(calibration);
            case bf_io::FilterType::EKF:
                return std::make_unique<ExtendedKalmanFilter>(calibration);
            case bf_io::FilterType::UKF:
                return std::make_unique<UnscentedKalmanFilter>(calibration);
            case bf_io::FilterType::PF:
                return std::make_unique<ParticleFilter>(calibration);
            case bf_io::FilterType::NONE:
            default:
                throw std::invalid_argument("BayesianFilterFactory::CreateFilter - Invalid filter type!");
        }
    }
}   // namespace bf

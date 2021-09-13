/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTER_FACTORY_INCLUDE_BAYESIAN_FILTER_FACTORY_HPP_
#define COMPONENTS_FILTER_FACTORY_INCLUDE_BAYESIAN_FILTER_FACTORY_HPP_

#include <memory>

#include "core_bayesian_filter.hpp"
#include "filter_calibration.hpp"
#include "filter_type.hpp"

namespace bf
{
    class BayesianFilterFactory
    {
        public:
            BayesianFilterFactory(void) = delete;
            BayesianFilterFactory(const BayesianFilterFactory & obj) = delete;
            BayesianFilterFactory(BayesianFilterFactory && obj) = delete;
            ~BayesianFilterFactory(void) = default;

            static std::unique_ptr<CoreBayesianFilter> CreateFilter(const bf_io::FilterType filter_type,
                const bf_io::FilterCalibration & calibration);
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTER_FACTORY_INCLUDE_BAYESIAN_FILTER_FACTORY_HPP_

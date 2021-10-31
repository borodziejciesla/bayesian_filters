/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_INCLUDE_PARTICLE_FILTER_HPP_
#define COMPONENTS_FILTERS_INCLUDE_PARTICLE_FILTER_HPP_

#include <memory>
#include <random>
#include <vector>

#include "core_bayesian_filter.hpp"
#include "filter_calibration.hpp"
#include "value_with_timestamp_and_covariance.hpp"

namespace bf
{
    class ParticleFilter : public CoreBayesianFilter
    {
        public:
            ParticleFilter(void) = delete;
            ParticleFilter(const ParticleFilter & obj) = delete;
            ParticleFilter(ParticleFilter && obj) = delete;
            ~ParticleFilter(void) = default;

            explicit ParticleFilter(const bf_io::FilterCalibration & calibration);

        protected:
            void Prediction(const float time_delta);
            void Correction(const bf_io::ValueWithTimestampAndCovariance & measurement);
        
        private:
            void SetWeightsBasedOnMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement);
            void NormalizeWeight(void);
            void Resampling(void);
            void MakeStateAndCovarianceEstimation(void);

            using WeightedSample = std::pair<float, Eigen::VectorXf>;

            size_t samples_number_ = 0u;
            float weights_sum_ = 0.0f;
            std::vector<WeightedSample> distribution_weighted_points_;
            std::vector<float> c_;

            std::default_random_engine generator_;
            std::unique_ptr<std::uniform_real_distribution<float>> uniform_distribution_;
            std::unique_ptr<std::normal_distribution<float>> normal_distribution_;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_INCLUDE_PARTICLE_FILTER_HPP_

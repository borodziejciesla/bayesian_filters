/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "particle_filter.hpp"

#include <algorithm>
#include <numeric>

namespace bf
{
    ParticleFilter::ParticleFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration)
        , samples_number_{dimension_ * 100u}
        , distribution_weighted_points_{std::vector<WeightedSample>(samples_number_)}
        , c_(std::vector<float>(samples_number_))
        , uniform_distribution_{std::make_unique<std::uniform_real_distribution<float>>(0.0f, 1.0f / static_cast<float>(samples_number_))} 
        , normal_distribution_{std::make_unique<std::normal_distribution<float>>(0.0f, 1.0f)} { // TODO
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [this](WeightedSample & weighted_sample) {
                weighted_sample.first = 1.0f / static_cast<float>(samples_number_);
                weighted_sample.second = Eigen::VectorXf::Random(dimension_);
            }
        );
    }

    ParticleFilter::StateWithCovariance ParticleFilter::Prediction(const float time_delta) {
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [time_delta,this](WeightedSample & weighted_sample) {
                weighted_sample.second = transition_(weighted_sample.second, time_delta); // Add path
            }
        );

        return ParticleFilter::StateWithCovariance();
    }

    ParticleFilter::StateWithCovariance ParticleFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
        const Eigen::VectorXf & predicted_state,
        const Eigen::MatrixXf & predicted_covariance) {
        std::ignore = measurement;
        std::ignore = predicted_state;
        std::ignore = predicted_covariance;
        SetWeightsBasedOnMeasurement();
        NormalizeWeight();
        Resampling();
        auto estimation = MakeStateAndCovarianceEstimation();

        return estimation;
    }

    void ParticleFilter::SetWeightsBasedOnMeasurement(void) {
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [this](WeightedSample & weighted_sample) {
                weighted_sample.first = 0.01f;
            }
        );
    }

    void ParticleFilter::NormalizeWeight(void) {
        float weights_sum = std::accumulate(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            0.0f,
            [](float accumulated, const WeightedSample & weighted_sample) {
                return accumulated + weighted_sample.first;
            }
        );

        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [weights_sum,this](WeightedSample & weighted_sample) {
                weighted_sample.first /= weights_sum;
            }
        );
    }

    void ParticleFilter::Resampling(void) {
        for (auto index = 1u; index < samples_number_; index++)
            c_.at(index) = c_.at(index - 1u) + distribution_weighted_points_.at(index).first;
        
        float u_1 = (*uniform_distribution_)(generator_);
        auto c_iterator = c_.begin();
        auto new_weight = 1.0f / static_cast<float>(samples_number_);

        for (auto index = 0u; index < samples_number_; index++) {
            auto u_index = u_1 * 0.0f + new_weight * static_cast<float>(index);

            while (u_index > *c_iterator)
                c_iterator++;

            auto i = (static_cast<size_t>(c_iterator - c_.begin()) < samples_number_) ? static_cast<size_t>(c_iterator - c_.begin()) : (samples_number_ - 1u);
            distribution_weighted_points_.at(index) = WeightedSample(new_weight, distribution_weighted_points_.at(i).second);
        }
    }

    ParticleFilter::StateWithCovariance ParticleFilter::MakeStateAndCovarianceEstimation(void) {
        /* Find mean value */
        auto mean_value = std::accumulate(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            static_cast<Eigen::VectorXf>(Eigen::VectorXf::Zero(dimension_)),
            [](Eigen::VectorXf accumulated_mean, WeightedSample sample_point) {
                return accumulated_mean + sample_point.first * sample_point.second;
            }
        );

        /* Estimate covariance */
        auto covariance_of_mean_value = std::accumulate(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(dimension_, dimension_)),
            [mean_value](Eigen::MatrixXf accumulated_covariance, WeightedSample sample_point) {
                auto difference = sample_point.second - mean_value;
                return accumulated_covariance + (sample_point.first * difference * difference.transpose());
            }
        );

        return std::tuple<Eigen::VectorXf, Eigen::MatrixXf>(mean_value, covariance_of_mean_value);
    }
}

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
#include <functional>
#include <numbers>
#include <numeric>
#include <iostream>

namespace bf
{
    ParticleFilter::ParticleFilter(const bf_io::FilterCalibration & calibration)
        : CoreBayesianFilter(calibration)
        , samples_number_{dimension_ * 500u}
        , distribution_weighted_points_{std::vector<WeightedSample>(samples_number_)}
        , c_(std::vector<float>(samples_number_))
        , uniform_distribution_{std::make_unique<std::uniform_real_distribution<float>>(0.0f, 1.0f / static_cast<float>(samples_number_))} {
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [this](WeightedSample & weighted_sample) {
                weighted_sample.first = 1.0f / static_cast<float>(samples_number_);
                weighted_sample.second = 10.0f * Eigen::VectorXf::Random(dimension_);
                for (auto index = 1u; index < dimension_; index += 2u)
                    weighted_sample.second(index) = 0.0f;
            }
        );

        for (auto index = 0u; index < dimension_; index++)
            normal_distribution_.push_back(std::make_unique<std::normal_distribution<float>>(0.0f, process_noise_covariance_(index, index)));
    }

    void ParticleFilter::Prediction(const float time_delta) {
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [time_delta,this](WeightedSample & weighted_sample) {
                Eigen::VectorXf noise = Eigen::VectorXf::Zero(dimension_);
                for (auto index = 0u; index < dimension_; index++)
                    noise(index) = (*normal_distribution_.at(index))(generator_);
                
                weighted_sample.second = transition_(weighted_sample.second, noise, time_delta);
            }
        );
    }

    void ParticleFilter::Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        SetWeightsBasedOnMeasurement(measurement);
        NormalizeWeight();
        MakeStateAndCovarianceEstimation();
        Resampling();
    }

    void ParticleFilter::SetWeightsBasedOnMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        auto [y, R] = ConvertMeasurement(measurement);
        weights_sum_ = 0.0f;
        
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [this,&y,&R](WeightedSample & weighted_sample) {
                // Gaussian distribution
                auto predicted_observation = observation_(weighted_sample.second);
                auto distance = predicted_observation - y;

                auto jac = observation_jacobian_(weighted_sample.second);
                auto sigma = static_cast<Eigen::MatrixXf>(jac * estimated_covariance_ * jac.transpose() + R);
                auto exp_argument = -0.5f * distance.transpose() * sigma.inverse() * distance;
                auto denumerator = std::pow(2.0f * std::numbers::pi_v<float>, 0.5f * static_cast<float>(dimension_) * std::sqrt(R.determinant()));

                weighted_sample.first = std::exp(exp_argument(0)) / denumerator;
                weights_sum_ += weighted_sample.first;

                // std::cout << "\n ============================== \n Point \n" << weighted_sample.second << "\n";
                // std::cout << "\n Predicted observation \n" << predicted_observation << "\n";
                // std::cout << "\n jac \n" << jac << "\n";
                // std::cout << "\n estimated_covariance_ \n" << estimated_covariance_ << "\n";
                // std::cout << "\n R \n" << R << "\n";
                // std::cout << "\n sigma \n" << sigma << "\n";
                // std::cout << "\n Y \n" << y << "\n";
                // std::cout << "\n distance \n" << distance << "\n";
                // std::cout << "\n Weight \n" << weighted_sample.first << "\n";
            }
        );
    }

    void ParticleFilter::NormalizeWeight(void) {
        std::for_each(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            [this](WeightedSample & weighted_sample) {
                weighted_sample.first /= weights_sum_;
            }
        );
    }

    void ParticleFilter::Resampling(void) {
        c_.at(0u) = 0.0f;
        for (auto index = 1u; index < samples_number_; index++)
            c_.at(index) = c_.at(index - 1u) + distribution_weighted_points_.at(index).first;
        std::sort(c_.begin(), c_.end(), std::less<float>());
        
        auto c_iterator = c_.begin();
        auto new_weight = 1.0f / static_cast<float>(samples_number_);
        auto u_0 = (*uniform_distribution_)(generator_);

        for (auto index = 0u; index < samples_number_; index++) {
            auto u_index = u_0 + new_weight * static_cast<float>(index);

            while (u_index > *c_iterator)
                c_iterator++;

            auto i = (static_cast<size_t>(c_iterator - c_.begin()) < samples_number_) ? static_cast<size_t>(c_iterator - c_.begin()) : (samples_number_ - 1u);
            distribution_weighted_points_.at(index) = WeightedSample(new_weight, distribution_weighted_points_.at(i).second);
        }
    }

    void ParticleFilter::MakeStateAndCovarianceEstimation(void) {
        /* Find mean value */
        estimated_state_ = std::accumulate(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            static_cast<Eigen::VectorXf>(Eigen::VectorXf::Zero(dimension_)),
            [this](Eigen::VectorXf accumulated_mean, WeightedSample sample_point) {
                // std::cout << "\n *************** \n Point \n" << sample_point.second << "\n";
                // std::cout << "\n Weight \n" << sample_point.first << "\n";
                return accumulated_mean + sample_point.second * sample_point.first;
            }
        );

        // std::cout << "\n +++++++++++++++++ \n ESTIMATION \n" << estimated_state_ << "\n";

        // auto max_element = std::max_element(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
        //     [](const WeightedSample & lhs, const WeightedSample & rhs) {
        //         return lhs.first < rhs.first;
        //     }
        // );
        // std::cout << "\n ################### \n Point \n" << (*max_element).second << "\n";
        // std::cout << "\n Weight \n" << (*max_element).first << "\n";


        /* Estimate covariance */
        estimated_covariance_ = std::accumulate(distribution_weighted_points_.begin(), distribution_weighted_points_.end(),
            static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Zero(dimension_, dimension_)),
            [this](Eigen::MatrixXf accumulated_covariance, WeightedSample sample_point) {
                auto difference = sample_point.second - estimated_state_;
                return accumulated_covariance + (sample_point.first * difference * difference.transpose());
            }
        );
    }
}   // namespace bf

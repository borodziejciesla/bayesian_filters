/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_INCLUDE_CORE_BAYESIAN_FILTER_HPP_
#define COMPONENTS_FILTERS_INCLUDE_CORE_BAYESIAN_FILTER_HPP_

#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

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
            virtual ~CoreBayesianFilter(void) = default;

            explicit CoreBayesianFilter(const bf_io::FilterCalibration & calibration);

            void RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement);
            const bf_io::ValueWithTimestampAndCovariance & GetEstimation(void);

        protected:
            using StateWithCovariance = std::tuple<Eigen::VectorXf, Eigen::MatrixXf>;

            virtual void Prediction(const float time_delta) = 0;
            virtual void Correction(const bf_io::ValueWithTimestampAndCovariance & measurement) = 0;

            StateWithCovariance ConvertMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement) const;

            bf_io::ValueWithTimestampAndCovariance state_;

            size_t dimension_ = 0u;
            size_t measurement_dimension_ = 0u;

            Eigen::VectorXf estimated_state_;
            Eigen::MatrixXf estimated_covariance_;

            Eigen::VectorXf predicted_state_;
            Eigen::MatrixXf predicted_covariance_;

            Eigen::MatrixXf process_noise_covariance_;

            std::function<Eigen::VectorXf(const Eigen::VectorXf & state,
                const std::optional<Eigen::VectorXf> & noise,
                const float time_delta)> transition_;
            std::function<Eigen::MatrixXf(const Eigen::VectorXf & state, const float time_delta)> transition_jacobian_;

            std::function<Eigen::VectorXf(const Eigen::VectorXf & state)> observation_;
            std::function<Eigen::MatrixXf(const Eigen::VectorXf & state)> observation_jacobian_;

        private:
            void ConvertEstimateToOutput(void);
            std::vector<float> ConvertVectorxToState(void) const;
            bf_io::Covariance ConvertMatrixToCovariance(void) const;

            float previous_timestamp_ = 0.0f;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_INCLUDE_CORE_BAYESIAN_FILTER_HPP_

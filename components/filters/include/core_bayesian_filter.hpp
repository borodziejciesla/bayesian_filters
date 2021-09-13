/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_
#define COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_

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
            ~CoreBayesianFilter(void) = default;

            explicit CoreBayesianFilter(const bf_io::FilterCalibration & calibration);

            void RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement);
            const bf_io::ValueWithTimestampAndCovariance & GetEstimation(void);

        protected:
            using StateWithCovariance = std::tuple<Eigen::VectorXf, Eigen::MatrixXf>;

            virtual StateWithCovariance Prediction(const float time_delta) = 0;
            virtual StateWithCovariance Correction(const bf_io::ValueWithTimestampAndCovariance & measurement,
                const Eigen::VectorXf & predicted_state,
                const Eigen::MatrixXf & predicted_covariance) = 0;

            StateWithCovariance ConvertMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement) const;

            bf_io::ValueWithTimestampAndCovariance state_;

            Eigen::VectorXf estimated_state_;
            Eigen::MatrixXf estimated_covariance_;

            std::function<Eigen::VectorXf(const Eigen::VectorXf & state, const float time_delta)> transition_;
            std::function<Eigen::MatrixXf(const Eigen::VectorXf & state)> transition_jacobian_;

            std::function<Eigen::VectorXf(const Eigen::VectorXf & state)> observation_;
            std::function<Eigen::MatrixXf(const Eigen::VectorXf & state)> observation_jacobian_;

        private:
            void ConvertEstimateToOutput(const Eigen::VectorXf & state, const Eigen::MatrixXf & covariance);
            std::vector<float> ConvertVectorxToState(const Eigen::VectorXf & state) const;
            bf_io::Covariance ConvertMatrixToCovariance(const Eigen::MatrixXf & covariance) const;

            double previous_timestamp_ = 0.0;
    };
}   // namespace bf

#endif  //  COMPONENTS_FILTERS_CORE_BAYESIAN_FILTER_HPP_

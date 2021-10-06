/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "core_bayesian_filter.hpp"

#include <stdexcept>

namespace bf
{
    CoreBayesianFilter::CoreBayesianFilter(const bf_io::FilterCalibration & calibration)
        : transition_{calibration.transition}
        , transition_jacobian_{calibration.transition_jacobian}
        , observation_{calibration.observation}
        , observation_jacobian_{calibration.observation_jacobian} {
        dimension_ = calibration.state_dimension_;
        measurement_dimension_ = calibration.measurement_dimension;

        estimated_state_ = Eigen::VectorXf::Zero(dimension_);
        estimated_covariance_ = Eigen::MatrixXf::Zero(dimension_, dimension_);
        for (auto index = 0u; index < dimension_; index++)
            estimated_covariance_(index, index) = 1.0e2f;
    }

    void CoreBayesianFilter::RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        auto time_delta = static_cast<float>(measurement.timestamp - previous_timestamp_);
        previous_timestamp_ = measurement.timestamp;
        
        auto [predicted_state, predicted_covariance] = Prediction(time_delta);
        auto [state, covariance] = Correction(measurement, predicted_state, predicted_covariance);

        ConvertEstimateToOutput(state, covariance);
        estimated_state_ = state;
        estimated_covariance_ = covariance;
    }
    
    const bf_io::ValueWithTimestampAndCovariance & CoreBayesianFilter::GetEstimation(void) {
        return state_;
    }

    CoreBayesianFilter::StateWithCovariance CoreBayesianFilter::ConvertMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement) const {
        auto size = measurement.state.size();
        
        Eigen::VectorXf measurementr_value;
        measurementr_value.resize(size);
        for (size_t idx = 0u; idx < size; idx++)
        {
            measurementr_value(idx) = measurement.state.at(idx);
            idx++;
        }

        Eigen::MatrixXf covariance(size, size);
        for (size_t diag_idx = 0u; diag_idx < size; diag_idx++)
            covariance(diag_idx, diag_idx) = measurement.covariance.diagonal.at(diag_idx);

        auto idx = 0u;
        for (auto r = 1u; r < size; r++)
        {
            for (auto c = 0u; c < r; c++)
            {
                covariance(r, c) = measurement.covariance.lower_triangle.at(idx);
                covariance(c, r) = measurement.covariance.lower_triangle.at(idx);
                idx++;
            }
        }

        return std::make_tuple(measurementr_value, covariance);
    }

    void CoreBayesianFilter::ConvertEstimateToOutput(const Eigen::VectorXf & state, const Eigen::MatrixXf & covariance) {
        state_.timestamp = previous_timestamp_;
        state_.state = ConvertVectorxToState(state);
        state_.covariance = ConvertMatrixToCovariance(covariance);
    }

    std::vector<float> CoreBayesianFilter::ConvertVectorxToState(const Eigen::VectorXf & state) const {
        std::vector<float> output(state.size());

        for (auto idx = 0u; idx < state.size(); idx++)
            output.at(idx) = state(idx);

        return output;
    }

    bf_io::Covariance CoreBayesianFilter::ConvertMatrixToCovariance(const Eigen::MatrixXf & covariance) const {
        bf_io::Covariance output;
        
        if (covariance.rows() != covariance.cols())
            throw std::invalid_argument("CoreBayesianFilter::ConvertMatrixToCovariance - invalid matrix size!");

        auto size = covariance.rows();

        output.diagonal.resize(size);
        output.lower_triangle.resize((size) * (size - 1u) / 2u);

        for (auto idx = 0u; idx < size; idx++)
           output.diagonal.at(idx) = covariance(idx, idx);

        auto idx = 0u;
        for (auto r = 1u; r < size; r++)
            for (auto c = 0u; c < r; c++)
                output.lower_triangle.at(idx++) = covariance(r, c);

        return output;
    }
}   // namespace bf

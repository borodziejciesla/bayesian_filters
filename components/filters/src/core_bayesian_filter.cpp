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
        : process_noise_covariance_{calibration.proccess_noise_covariance}
        , transition_{calibration.transition}
        , transition_jacobian_{calibration.transition_jacobian}
        , observation_{calibration.observation}
        , observation_jacobian_{calibration.observation_jacobian} {
        dimension_ = calibration.state_dimension;
        measurement_dimension_ = calibration.measurement_dimension;

        estimated_state_ = Eigen::VectorXf::Random(dimension_);
        estimated_covariance_ = 10.0f * Eigen::MatrixXf::Identity(dimension_, dimension_);
    }

    void CoreBayesianFilter::RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement) {
        auto time_delta = static_cast<float>(measurement.timestamp - previous_timestamp_);
        previous_timestamp_ = measurement.timestamp;
        
        Prediction(time_delta);
        Correction(measurement);

        ConvertEstimateToOutput();
    }
    
    const bf_io::ValueWithTimestampAndCovariance & CoreBayesianFilter::GetEstimation(void) {
        return state_;
    }

    CoreBayesianFilter::StateWithCovariance CoreBayesianFilter::ConvertMeasurement(const bf_io::ValueWithTimestampAndCovariance & measurement) const {
        auto size = measurement.state.size();
        
        Eigen::VectorXf measurementr_value;
        measurementr_value.resize(size);
        for (size_t idx = 0u; idx < size; idx++)
            measurementr_value(idx) = measurement.state.at(idx);

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

    void CoreBayesianFilter::ConvertEstimateToOutput(void) {
        state_.timestamp = previous_timestamp_;
        state_.state = ConvertVectorxToState();
        state_.covariance = ConvertMatrixToCovariance();
    }

    std::vector<float> CoreBayesianFilter::ConvertVectorxToState(void) const {
        std::vector<float> output(estimated_state_.size());

        for (auto idx = 0u; idx < estimated_state_.size(); idx++)
            output.at(idx) = estimated_state_(idx);

        return output;
    }

    bf_io::Covariance CoreBayesianFilter::ConvertMatrixToCovariance(void) const {
        bf_io::Covariance output;
        
        if (estimated_covariance_.rows() != estimated_covariance_.cols())
            throw std::invalid_argument("CoreBayesianFilter::ConvertMatrixToCovariance - invalid matrix size!");

        auto size = estimated_covariance_.rows();

        output.diagonal.resize(size);
        output.lower_triangle.resize((size) * (size - 1u) / 2u);

        for (auto idx = 0u; idx < size; idx++)
           output.diagonal.at(idx) = estimated_covariance_(idx, idx);

        auto idx = 0u;
        for (auto r = 1u; r < size; r++)
            for (auto c = 0u; c < r; c++)
                output.lower_triangle.at(idx++) = estimated_covariance_(r, c);

        return output;
    }
}   // namespace bf

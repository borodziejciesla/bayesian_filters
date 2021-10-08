#include <iostream>

#include "matplotlibcpp.h"

#include "bayesian_filter.hpp"

namespace plt = matplotlibcpp;

int main(void) {
    bf_io::FilterCalibration calibrations;

    /*
     * Create Model:
     * x(k+1) = [x_1 + T * x_2; sin(x_2) * x2]
     * y(k) = x_1
     */
    calibrations.transition = [](const Eigen::VectorXf & state, const float time_delta) {
        Eigen::VectorXf transformed_state(2);
        transformed_state(0) = state(0) + time_delta * state(1);
        transformed_state(1) = std::sin(state(1)) * state(1);

        return transformed_state;
    };

    calibrations.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
        Eigen::MatrixXf jacobian(2, 2);
        jacobian(0, 0) = 1.0f;
        jacobian(0, 1) = time_delta;
        jacobian(1, 0) = 0.0f;
        jacobian(1, 1) = std::sin(state(1)) + std::cos(state(1)) * state(1);

        return jacobian;
    };

    calibrations.observation = [](const Eigen::VectorXf & state) {
        Eigen::VectorXf observation(1, 1);
        observation(0, 0) = state(0);

        return observation;
    };

    calibrations.observation_jacobian = [](const Eigen::VectorXf & state) {
        std::ignore = state;
        
        Eigen::MatrixXf jacobian(1, 2);
        jacobian(0, 0) = 1.0f;
        jacobian(0, 1) = 0.0f;

        return jacobian;
    };

    calibrations.state_dimension_ = 2u;
    calibrations.measurement_dimension = 1u;
    calibrations.proccess_noise_covariance = Eigen::MatrixXf::Ones(2, 2);

    /* Create filters */
    bf::BayesianFilter ekf_filter(bf_io::FilterType::EKF, calibrations);
    bf::BayesianFilter ukf_filter(bf_io::FilterType::UKF, calibrations);

    /* Run filters */
    bf_io::ValueWithTimestampAndCovariance measurement;
    measurement.timestamp = 0.0;
    measurement.state = {10.0f};
    measurement.covariance.diagonal = {1.0f};

    std::vector<double> ekf_x1(100u);
    std::vector<double> ekf_x2(100u);
    std::vector<double> ukf_x1(100u);
    std::vector<double> ukf_x2(100u);
    std::vector<double> timestamps(100u);
    std::vector<double> meas(100u);

    for (auto index = 0u; index < 100u; index++)
    {        
        measurement.timestamp += 0.01f;

        timestamps.at(index) = measurement.timestamp;
        meas.at(index) = static_cast<double>( measurement.state.at(0u));
        
        // EKF
        ekf_filter.RunFilter(measurement);
        auto ekf_result = ekf_filter.GetEstimation();
        ekf_x1.at(index) = ekf_result.state.at(0);
        ekf_x2.at(index) = ekf_result.state.at(1);

        // UKF
        ukf_filter.RunFilter(measurement);
        auto ukf_result = ukf_filter.GetEstimation();
        ukf_x1.at(index) = ukf_result.state.at(0);
        ukf_x2.at(index) = ukf_result.state.at(1);
    }

    /* Plots */
    plt::figure();
    plt::plot(timestamps, ekf_x1, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_x1, "r", {{"label", "UKF"}});
    plt::plot(timestamps, meas, "k+", {{"label", "Measurement"}});
    plt::title("x1");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("x1 [-]");
    plt::show();
    
    plt::figure();
    plt::plot(timestamps, ekf_x2, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_x2, "r", {{"label", "UKF"}});
    plt::title("x2");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("x2 [-]");
    plt::show();

    return EXIT_SUCCESS;
}

#include <iostream>
#include <cmath>
#include <random>

#include "matplotlibcpp.h"

#include "bayesian_filter.hpp"

namespace plt = matplotlibcpp;

int main(void) {
    bf_io::FilterCalibration calibrations;

    /*
     * Create Model:
     * X(k+1) = [x + T * vx; vx; y + T * vy; vy]
     * Y(k) = [sqrt(x^2 + y^2); arctan(y/x)]
     */
    calibrations.transition = [](const Eigen::VectorXf & state, const float time_delta) {
        auto x = state(0);
        auto vx = state(1);
        auto y = state(2);
        auto vy = state(3);

        auto predicted_x = x + time_delta * vx;
        auto predicted_vx = vx;
        auto predicted_y = y + time_delta * vy;
        auto predicted_vy = vy;

        Eigen::VectorXf transformed_state(4u);
        transformed_state(0) = predicted_x;
        transformed_state(1) = predicted_vx;
        transformed_state(2) = predicted_y;
        transformed_state(3) = predicted_vy;

        return transformed_state;
    };

    calibrations.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
        std::ignore = state;

        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Identity(4u, 4u);
        jacobian(0, 1) = time_delta;
        jacobian(2, 3) = time_delta;

        return jacobian;
    };

    calibrations.observation = [](const Eigen::VectorXf & state) {
        auto x = state(0);
        auto y = state(2);

        auto range = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        auto azimuth = std::atan2(y, x);

        Eigen::VectorXf observation(2);
        observation(0) = range;
        observation(1) = azimuth;

        return observation;
    };

    calibrations.observation_jacobian = [](const Eigen::VectorXf & state) {
        auto x = state(0);
        auto y = state(2);
        
        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(2u, 4u);
        if ((std::abs(x) < 1e-5f) && (std::abs(y) < 1e-5f)) {
            jacobian(0, 0) = 1.0f;
            jacobian(0, 1) = 1.0f;
            jacobian(1, 2) = 1.0f;
            jacobian(1, 3) = 1.0f;
        } else {
            auto range_sqr = std::pow(x, 2) + std::pow(y, 2);
            auto range = std::sqrt(range_sqr);
            jacobian(0, 0) = x / range;
            jacobian(0, 2) = y / range;
            jacobian(1, 0) = -y / range_sqr;
            jacobian(1, 2) = x / range_sqr;
        }

        return jacobian;
    };

    calibrations.state_dimension_ = 4u;
    calibrations.measurement_dimension = 2u;
    calibrations.proccess_noise_covariance = 2.0f * static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Identity(4, 4));
    
    /* Create filters */
    bf::BayesianFilter ekf_filter(bf_io::FilterType::EKF, calibrations);
    bf::BayesianFilter ukf_filter(bf_io::FilterType::UKF, calibrations);

    /* Run filters */
    bf_io::ValueWithTimestampAndCovariance measurement;
    measurement.timestamp = 0.0;
    measurement.state = {10.0f, 10.0f};
    measurement.covariance.diagonal = {0.1f, 0.01f};
    measurement.covariance.lower_triangle = {0.0f};

    constexpr size_t size = 1000u;

    std::vector<double> ref_x(size, 10.0f);
    std::vector<double> ref_y(size, 0.0f);
    std::vector<double> ekf_x(size);
    std::vector<double> ekf_vx(size);
    std::vector<double> ekf_y(size);
    std::vector<double> ekf_vy(size);
    std::vector<double> ukf_x(size);
    std::vector<double> ukf_y(size);
    std::vector<double> timestamps(size);
    std::vector<double> meas_x(size);
    std::vector<double> meas_y(size);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution_range(0.0f, 0.1f);
    std::normal_distribution<float> distribution_azimuth(0.0f, 0.01f);

    for (auto index = 0u; index < size; index++)
    {        
        measurement.timestamp += 0.01f;
        auto x = 10.0f;
        auto y = 10.0f;
        measurement.state.at(0) = std::sqrt(std::pow(x, 2) + std::pow(y, 2)) + distribution_range(generator);
        measurement.state.at(1) = std::atan2(y, x) + distribution_azimuth(generator);

        timestamps.at(index) = measurement.timestamp;
        meas_x.at(index) = measurement.state.at(0) * std::cos(measurement.state.at(1));
        meas_y.at(index) = measurement.state.at(0) * std::sin(measurement.state.at(1));
        
        // EKF
        ekf_filter.RunFilter(measurement);
        auto ekf_result = ekf_filter.GetEstimation();
        ekf_x.at(index) = static_cast<double>(ekf_result.state.at(0));
        ekf_vx.at(index) = static_cast<double>(ekf_result.state.at(1));
        ekf_y.at(index) = static_cast<double>(ekf_result.state.at(2));
        ekf_vy.at(index) = static_cast<double>(ekf_result.state.at(3));

        // UKF
        ukf_filter.RunFilter(measurement);
        auto ukf_result = ukf_filter.GetEstimation();
        ukf_x.at(index) = ukf_result.state.at(0);
        ukf_y.at(index) = ukf_result.state.at(1);
    }

    /* Plots */
    plt::figure();
    plt::plot(timestamps, ref_x, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_x, "b", {{"label", "EKF"}});
    //plt::plot(timestamps, ukf_x, "r", {{"label", "UKF"}});
    plt::plot(timestamps, meas_x, "k+", {{"label", "Measurement"}});
    plt::title("x");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("x [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vx, "b", {{"label", "EKF"}});
    //plt::plot(timestamps, ukf_x, "r", {{"label", "UKF"}});
    plt::title("vx");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vx [m/s]");
    plt::show();
    
    plt::figure();
    plt::plot(timestamps, ref_y, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_y, "b", {{"label", "EKF"}});
    //plt::plot(timestamps, ukf_y, "r", {{"label", "UKF"}});
    plt::plot(timestamps, meas_y, "k+", {{"label", "Measurement"}});
    plt::title("y");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("y [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vy, "b", {{"label", "EKF"}});
    //plt::plot(timestamps, ukf_x, "r", {{"label", "UKF"}});
    plt::title("vy");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vy [m/s]");
    plt::show();

    return EXIT_SUCCESS;
}

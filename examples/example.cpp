/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include <iostream>
#include <cmath>
#include <random>
#include <string>

#include "matplotlibcpp.h"

#include "bayesian_filter.hpp"

namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cout << "Invalid Number of arguments!";
        return EXIT_FAILURE;
    }

    std::string argument_1(argv[1]);
    const auto size = std::stoi(argument_1);
    std::string argument_2(argv[2]);
    const float range_std = std::stof(argument_2);
    std::string argument_3(argv[3]);
    const float azimuth_std = std::stof(argument_3);

    bf_io::FilterCalibration calibrations;

    /*
     * Create Model:
     * X(k+1) = [x + T * vx; vx; y + T * vy; vy]
     * Y(k) = [sqrt(x^2 + y^2); arctan(y/x)]
     */
    calibrations.transition = [](const Eigen::VectorXf & state,
        const std::optional<Eigen::VectorXf> & noise,
        const float time_delta) {
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

        if (noise.has_value()) {
            auto noise_value = noise.value();
            transformed_state(0) += time_delta * noise_value(0) + std::pow(time_delta, 2.0f) * noise_value(1);
            transformed_state(1) += time_delta * noise_value(1);
            transformed_state(2) += time_delta * noise_value(2) + std::pow(time_delta, 2.0f) * noise_value(3);
            transformed_state(3) += time_delta * noise_value(3);
        }

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
            auto range_sqr = std::pow(x, 2.0f) + std::pow(y, 2.0f);
            auto range = std::sqrt(range_sqr);
            jacobian(0, 0) = x / range;
            jacobian(0, 2) = y / range;
            jacobian(1, 0) = -y / range_sqr;
            jacobian(1, 2) = x / range_sqr;
        }

        return jacobian;
    };

    calibrations.state_dimension = 4u;
    calibrations.measurement_dimension = 2u;
    calibrations.proccess_noise_covariance = static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Identity(4, 4));
    calibrations.proccess_noise_covariance(0, 0) = 10.0f;
    calibrations.proccess_noise_covariance(1, 1) = 10.0f;
    calibrations.proccess_noise_covariance(2, 2) = 1.0f;
    calibrations.proccess_noise_covariance(3, 3) = 1.0f;

    /* Create filters */
    bf::BayesianFilter ekf_filter(bf_io::FilterType::EKF, calibrations);
    bf::BayesianFilter ukf_filter(bf_io::FilterType::UKF, calibrations);
    bf::BayesianFilter pf_filter(bf_io::FilterType::PF, calibrations);

    /* Run filters */
    bf_io::ValueWithTimestampAndCovariance measurement;
    measurement.timestamp = 0.0;
    measurement.state = {10.0f, 10.0f};
    measurement.covariance.diagonal = {std::pow(range_std, 2.0f), std::pow(azimuth_std, 2.0f)};
    measurement.covariance.lower_triangle = {0.0f};

    std::vector<double> ref_x(size, 10.0f);
    std::vector<double> ref_y(size, 10.0f);
    std::vector<double> ekf_x(size);
    std::vector<double> ekf_vx(size);
    std::vector<double> ekf_y(size);
    std::vector<double> ekf_vy(size);
    std::vector<double> ukf_x(size);
    std::vector<double> ukf_vx(size);
    std::vector<double> ukf_y(size);
    std::vector<double> ukf_vy(size);
    std::vector<double> pf_x(size);
    std::vector<double> pf_vx(size);
    std::vector<double> pf_y(size);
    std::vector<double> pf_vy(size);
    std::vector<double> timestamps(size);
    std::vector<double> meas_x(size);
    std::vector<double> meas_y(size);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution_range(0.0f, range_std);
    std::normal_distribution<float> distribution_azimuth(0.0f, azimuth_std);

    for (auto index = 0; index < size; index++)
    {
        measurement.timestamp += 0.01f;
        auto x = static_cast<float>(ref_x.at(index));
        auto y = static_cast<float>(ref_y.at(index));
        measurement.state.at(0) = std::sqrt(std::pow(x, 2.0f) + std::pow(y, 2.0f)) + distribution_range(generator);
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
        ukf_vx.at(index) = ukf_result.state.at(1);
        ukf_y.at(index) = ukf_result.state.at(2);
        ukf_vy.at(index) = ukf_result.state.at(3);

        // PF
        pf_filter.RunFilter(measurement);
        auto pf_result = pf_filter.GetEstimation();
        pf_x.at(index) = pf_result.state.at(0);
        pf_vx.at(index) = pf_result.state.at(1);
        pf_y.at(index) = pf_result.state.at(2);
        pf_vy.at(index) = pf_result.state.at(3);
    }

    /* Plots */
    plt::figure();
    plt::plot(timestamps, ref_x, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_x, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_x, "r", {{"label", "UKF"}});
    plt::plot(timestamps, pf_x, "g", {{"label", "PF"}});
    plt::plot(timestamps, meas_x, "k.", {{"label", "Measurement"}});
    plt::title("x");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("x [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vx, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_vx, "r", {{"label", "UKF"}});
    plt::plot(timestamps, pf_vx, "g", {{"label", "PF"}});
    plt::title("vx");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vx [m/s]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ref_y, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_y, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_y, "r", {{"label", "UKF"}});
    plt::plot(timestamps, pf_y, "g", {{"label", "PF"}});
    plt::plot(timestamps, meas_y, "k.", {{"label", "Measurement"}});
    plt::title("y");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("y [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vy, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_vy, "r", {{"label", "UKF"}});
    plt::plot(timestamps, pf_vy, "g", {{"label", "PF"}});
    plt::title("vy");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vy [m/s]");
    plt::show();

    plt::figure();
    plt::plot(meas_x, meas_y, "k.", {{"label", "Measurement"}});
    plt::plot(ekf_x, ekf_y, "b.", {{"label", "EKF"}});
    plt::plot(ukf_x, ukf_y, "r.", {{"label", "UKF"}});
    plt::plot(pf_x, pf_y, "g.", {{"label", "PF"}});
    plt::plot(ref_x, ref_y, "m--", {{"label", "Reference"}});
    plt::title("Trajectory");
    plt::grid();
    plt::legend();
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::show();

    return EXIT_SUCCESS;
}

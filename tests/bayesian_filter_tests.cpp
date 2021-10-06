#include "gtest/gtest.h"

#include <tuple>

#include "bayesian_filter.hpp"

class BayesianFilterTests : public ::testing::TestWithParam<bf_io::FilterType>
{
    protected:
        bf_io::FilterCalibration calibrations_;

        void CreateSimpleModel(void) {
            calibrations_.transition = [](const Eigen::VectorXf & state, const float time_delta) {
                std::ignore = state;
                std::ignore = time_delta;
                return Eigen::VectorXf::Zero(1);
            };

            calibrations_.transition_jacobian = [](const Eigen::VectorXf & state) {
                std::ignore = state;
                return Eigen::MatrixXf::Ones(1, 1);
            };

            calibrations_.observation = [](const Eigen::VectorXf & state) {
                std::ignore = state;
                return Eigen::VectorXf::Zero(1);
            };

            calibrations_.observation_jacobian = [](const Eigen::VectorXf & state) {
                std::ignore = state;
                return Eigen::MatrixXf::Ones(1, 1);
            };

            calibrations_.state_dimension_ = 1u;
            calibrations_.measurement_dimension = 1u;
            calibrations_.proccess_noise_covariance = Eigen::MatrixXf::Ones(1, 1);
        }
};

TEST_P(BayesianFilterTests, TestConstructor) {
    auto filter_type = GetParam();
    CreateSimpleModel();

    if (filter_type != bf_io::FilterType::NONE)
        EXPECT_NO_THROW(bf::BayesianFilter filter(filter_type, calibrations_));
    else
        EXPECT_THROW(bf::BayesianFilter filter(filter_type, calibrations_), std::invalid_argument);
}

TEST_P(BayesianFilterTests, TestZeroInput) {
    auto filter_type = GetParam();
    CreateSimpleModel();

    if (filter_type != bf_io::FilterType::NONE) {
        bf::BayesianFilter filter(filter_type, calibrations_);

        bf_io::ValueWithTimestampAndCovariance measurement;
        measurement.timestamp = 0.0;
        measurement.state = {0.0f};
        measurement.covariance.diagonal = {100.0f};

        filter.RunFilter(measurement);

        auto result = filter.GetEstimation();

        EXPECT_DOUBLE_EQ(result.timestamp, measurement.timestamp);
        EXPECT_TRUE(measurement.state.size() == result.state.size());
        for (auto idx = 0u; idx < measurement.state.size(); idx++)
            EXPECT_NEAR(measurement.state.at(idx), result.state.at(idx), 1.0e-3f);
    } else {
        EXPECT_THROW(bf::BayesianFilter filter(filter_type, calibrations_), std::invalid_argument);
    }
}

INSTANTIATE_TEST_CASE_P(
    BayesianFilterTestsSet,
    BayesianFilterTests,
    ::testing::Values(
        bf_io::FilterType::KF, bf_io::FilterType::EKF, bf_io::FilterType::UKF, bf_io::FilterType::NONE
    )
);

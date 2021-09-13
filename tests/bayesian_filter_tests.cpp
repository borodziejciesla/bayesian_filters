#include "gtest/gtest.h"

#include "bayesian_filter.hpp"

class BayesianFilterTests : public ::testing::TestWithParam<bf_io::FilterType>
{
    protected:
        bf_io::FilterCalibration calibrations_;
};

TEST_P(BayesianFilterTests, TestConstructor) {
    auto filter_type = GetParam();

    calibrations_.transition = [](const Eigen::VectorXf & state, const float time_delta) {
        return Eigen::VectorXf::Zero(1);
    };

    calibrations_.transition_jacobian = [](const Eigen::VectorXf & state) {
        return Eigen::VectorXf::Ones(1);
    };

    calibrations_.observation = [](const Eigen::VectorXf & state) {
        return Eigen::VectorXf::Zero(1);
    };

    calibrations_.observation_jacobian = [](const Eigen::VectorXf & state) {
        return Eigen::VectorXf::Ones(1);
    };


    if (filter_type != bf_io::FilterType::NONE)
        EXPECT_NO_THROW(bf::BayesianFilter filter(filter_type, calibrations_));
    else
        EXPECT_THROW(bf::BayesianFilter filter(filter_type, calibrations_), std::invalid_argument);
}

INSTANTIATE_TEST_CASE_P(
    BayesianFilterTestsSet,
    BayesianFilterTests,
    ::testing::Values(
        bf_io::FilterType::KF, bf_io::FilterType::EKF, bf_io::FilterType::UKF, bf_io::FilterType::NONE
    )
);

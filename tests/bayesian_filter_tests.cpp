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

            calibrations_.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
                std::ignore = state;
                std::ignore = time_delta;
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

            calibrations_.state_dimension = 1u;
            calibrations_.measurement_dimension = 1u;
            calibrations_.proccess_noise_covariance = Eigen::MatrixXf::Ones(1, 1);
        }

        void CreateComplexModel(void) {
            /*
             * Create Model:
             * x(k+1) = [x_1 + T * x_2; cos(x_2)]
             * y(k) = x_1
             */
            calibrations_.transition = [](const Eigen::VectorXf & state, const float time_delta) {
                Eigen::VectorXf transformed_state(2);
                transformed_state(0) = state(0) + time_delta * state(1);
                transformed_state(1) = std::cos(state(1));

                return transformed_state;
            };

            calibrations_.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
                Eigen::MatrixXf jacobian(2, 2);
                jacobian(0, 0) = 1.0f;
                jacobian(0, 1) = time_delta;
                jacobian(1, 0) = 0.0f;
                jacobian(1, 1) = std::sin(state(1));

                return jacobian;
            };

            calibrations_.observation = [](const Eigen::VectorXf & state) {
                Eigen::VectorXf observation(1, 1);
                observation(0, 0) = state(0);

                return observation;
            };

            calibrations_.observation_jacobian = [](const Eigen::VectorXf & state) {
                std::ignore = state;
                
                Eigen::MatrixXf jacobian(1, 2);
                jacobian(0, 0) = 1.0f;
                jacobian(0, 1) = 0.0f;

                return jacobian;
            };

            calibrations_.state_dimension = 2u;
            calibrations_.measurement_dimension = 1u;
            calibrations_.proccess_noise_covariance = Eigen::MatrixXf::Ones(2, 2);
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

TEST_P(BayesianFilterTests, TestConstantNonZeroInput) {
    auto filter_type = GetParam();
    CreateComplexModel();

    if (filter_type != bf_io::FilterType::NONE) {
        bf::BayesianFilter filter(filter_type, calibrations_);

        bf_io::ValueWithTimestampAndCovariance measurement;
        measurement.timestamp = 0.0;
        measurement.state = {1.0f};
        measurement.covariance.diagonal = {100.0f};

        for (auto index = 0u; index < 100u; index++)
        {        
            measurement.timestamp += 0.01f;

            filter.RunFilter(measurement);
            auto result = filter.GetEstimation();

            EXPECT_DOUBLE_EQ(result.timestamp, measurement.timestamp);
            EXPECT_EQ(result.state.size(), 2u);
            //for (auto idx = 0u; idx < measurement.state.size(); idx++)
            //    EXPECT_NEAR(measurement.state.at(idx), result.state.at(idx), 1.0e-3f);
        }
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

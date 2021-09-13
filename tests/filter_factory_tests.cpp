#include "gtest/gtest.h"

#include "bayesian_filter_factory.hpp"

class DealiaserTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}

        bf_io::FilterCalibration calibrations_;
};

/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_IO_VALUE_WITH_TIMESTAMP_AND_COVARIANCE_HPP_
#define INCLUDE_IO_VALUE_WITH_TIMESTAMP_AND_COVARIANCE_HPP_

#include "covariance.hpp"

namespace bf_io
{
    struct ValueWithTimestampAndCovariance
    {
        double timestamp;
        
        std::vector<float> state;
        Covariance covariance;
    };
}   // namespace bf_io

#endif  //  INCLUDE_IO_VALUE_WITH_TIMESTAMP_AND_COVARIANCE_HPP_

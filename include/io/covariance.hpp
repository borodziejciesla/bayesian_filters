/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_IO_COVARIANCE_HPP_
#define INCLUDE_IO_COVARIANCE_HPP_

#include <vector>

namespace bf_io
{
    struct Covariance
    {
        std::vector<float> diagonal;
        std::vector<float> lower_triangle;
    };
}   // namespace bf_io

#endif  //  INCLUDE_IO_COVARIANCE_HPP_

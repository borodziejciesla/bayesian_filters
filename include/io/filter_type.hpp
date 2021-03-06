/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_IO_FILTER_TYPE_HPP_
#define INCLUDE_IO_FILTER_TYPE_HPP_

namespace bf_io
{
    enum class FilterType
    {
        KF = 0u,
        EKF = 1u,
        UKF = 2u,
        PF = 3u,
        NONE = 4u
    };
}   // namespace bf_io

#endif  //  INCLUDE_IO_FILTER_TYPE_HPP_

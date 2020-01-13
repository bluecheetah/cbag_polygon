// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POINT_CONCEPT_H
#define CBAG_POLYGON_POINT_CONCEPT_H

#include <cbag/polygon/enum.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {

template <typename T> struct coordinate_traits {};

template <> struct coordinate_traits<int32_t> { using area_type = int64_t; };

template <typename T, IsPoint<T> = 0> coord_type<T> get(const T &obj, orientation_2d orient) {
    return get_traits_t<T>::get(obj, orient);
}

template <typename T, IsPoint<T> = 0> coord_type<T> x(const T &obj) {
    return get_traits_t<T>::get(obj, orientation_2d::X);
}

template <typename T, IsPoint<T> = 0> coord_type<T> y(const T &obj) {
    return get_traits_t<T>::get(obj, orientation_2d::Y);
}

template <typename T, IsPoint<T> = 0> void set(T &obj, orientation_2d orient, coord_type<T> coord) {
    return get_traits_t<T>::set(obj, orient, coord);
}

template <typename T1, typename T2, IsPoint<T1> = 0, IsPoint<T2> = 0>
T1 &operator+=(T1 &lhs, const T2 &rhs) {
    set(lhs, orientation_2d::X, x(lhs) + x(rhs));
    set(lhs, orientation_2d::Y, y(lhs) + y(rhs));
    return lhs;
}

template <typename T1, typename T2, IsPoint<T1> = 0, IsPoint<T2> = 0>
T1 &operator-=(T1 &lhs, const T2 &rhs) {
    set(lhs, orientation_2d::X, x(lhs) - x(rhs));
    set(lhs, orientation_2d::Y, y(lhs) - y(rhs));
    return lhs;
}

template <typename T, IsPoint<T> = 0>
T &move_by_orient(T &lhs, orientation_2d orient, coord_type<T> dt, coord_type<T> dp) {
    auto perp = perpendicular(orient);
    set(lhs, orient, get(lhs, orient) + dt);
    set(lhs, perp, get(lhs, perp) + dp);
    return lhs;
}

template <typename T, IsPoint<T> = 0> T &move_by(T &lhs, coord_type<T> dx, coord_type<T> dy) {
    return move_by_orient(lhs, orientation_2d::X, dx, dy);
}

} // namespace polygon
} // namespace cbag

#endif

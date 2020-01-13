// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_INTERVAL_CONCEPT_H
#define CBAG_POLYGON_INTERVAL_CONCEPT_H

#include <type_traits>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {

template <typename T, IsInterval<T> = 0> coord_type<T> get(const T &intv, direction_1d dir) {
    return get_traits_t<T>::get(intv, dir);
}

template <typename T, IsInterval<T> = 0> coord_type<T> lower(const T &interval) {
    return get(interval, direction_1d::LOW);
}

template <typename T, IsInterval<T> = 0> coord_type<T> upper(const T &interval) {
    return get(interval, direction_1d::HIGH);
}

template <typename T, IsInterval<T> = 0> coord_type<T> center(const T &interval) {
    return (lower(interval) + upper(interval)) / 2;
}

template <typename T, IsInterval<T> = 0> bool is_physical(const T &interval) {
    return upper(interval) > lower(interval);
}

template <typename T, IsInterval<T> = 0> bool is_valid(const T &interval) {
    return upper(interval) >= lower(interval);
}

template <typename T, IsInterval<T> = 0>
void set(const T &intv, direction_1d dir, coord_type<T> value) {
    get_traits_t<T>::set(intv, dir, value);
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
bool operator<(const T1 &lhs, const T2 &rhs) {
    return upper(lhs) <= lower(rhs);
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
bool operator>(const T1 &lhs, const T2 &rhs) {
    return rhs < lhs;
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
bool operator>=(const T1 &lhs, const T2 &rhs) {
    return !(lhs < rhs);
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
bool operator<=(const T1 &lhs, const T2 &rhs) {
    return !(rhs < lhs);
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
T1 &operator|=(T1 &lhs, const T2 &rhs) {
    if (is_valid(rhs)) {
        if (is_valid(lhs)) {
            set(lhs, direction_1d::LOWER, std::min(lower(lhs), lower(rhs)));
            set(lhs, direction_1d::UPPER, std::max(upper(lhs), upper(rhs)));
        } else {
            set(lhs, direction_1d::LOWER, lower(rhs));
            set(lhs, direction_1d::UPPER, upper(rhs));
        }
    }
    return lhs;
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
T1 &operator&=(T1 &lhs, const T2 &rhs) {
    if (is_valid(rhs)) {
        if (is_valid(lhs)) {
            set(lhs, direction_1d::LOWER, std::max(lower(lhs), lower(rhs)));
            set(lhs, direction_1d::UPPER, std::min(upper(lhs), upper(rhs)));
        }
    } else {
        set(lhs, direction_1d::LOWER, lower(rhs));
        set(lhs, direction_1d::UPPER, upper(rhs));
    }
    return lhs;
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
T1 &operator+=(T1 &lhs, const T2 &rhs) {
    return lhs |= rhs;
}

template <typename T1, typename T2, IsInterval<T1> = 0, IsInterval<T2> = 0>
T1 &operator*=(T1 &lhs, const T2 &rhs) {
    return lhs &= rhs;
}

} // namespace polygon
} // namespace cbag

#endif

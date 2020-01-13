// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_MATH_H
#define CBAG_POLYGON_MATH_H

#include <climits>
#include <cstdint>

namespace cbag {
namespace polygon {
namespace math {

// source: https://graphics.stanford.edu/~seander/bithacks.html
inline constexpr uint8_t reverse_bits(uint8_t b) noexcept {
    return (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

inline constexpr uint8_t rotl8(uint8_t n, unsigned int c) noexcept {
    const unsigned int mask = (CHAR_BIT * sizeof(n) - 1); // assumes width is a power of 2.
    c &= mask;
    return (n << c) | (n >> ((-c) & mask));
}

inline constexpr uint8_t rotr8(uint8_t n, unsigned int c) noexcept {
    const unsigned int mask = (CHAR_BIT * sizeof(n) - 1);
    c &= mask;
    return (n >> c) | (n << ((-c) & mask));
}

// only works on arithmetic right shift architectures
inline constexpr int floor2(int a) noexcept { return a >> 1; }

template <typename T> inline constexpr int sign_diff(T a, T b) noexcept {
    return (b < a) - (a < b);
}

} // namespace math
} // namespace polygon
} // namespace cbag

#endif

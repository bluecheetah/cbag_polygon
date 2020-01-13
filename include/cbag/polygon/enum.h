// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_ENUM_H
#define CBAG_POLYGON_ENUM_H

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace cbag {

enum class polygon_set_type : uint16_t {
    POLY90 = 0,
    POLY45 = 1,
    POLY = 2,
};

inline std::string to_string(polygon_set_type t) {
    switch (t) {
    case polygon_set_type::POLY90:
        return "POLY90";
    case polygon_set_type::POLY45:
        return "POLY45";
    case polygon_set_type::POLY:
        return "POLY";
    default:
        throw std::runtime_error("Unsupported poly set type: " +
                                 std::to_string(static_cast<int>(t)));
    }
}

inline polygon_set_type max(polygon_set_type lhs, polygon_set_type rhs) noexcept {
    auto code = std::max(static_cast<uint16_t>(lhs), static_cast<uint16_t>(rhs));
    return static_cast<polygon_set_type>(code);
}

enum class direction_1d : uint16_t {
    LOW = 0,
    HIGH = 1,
    LOWER = 0,
    UPPER = 1,
    CLOCKWISE = 0,
    COUNTERCLOCKWISE = 1,
};

inline uint16_t to_int(direction_1d dir) noexcept { return static_cast<int>(dir); }

inline direction_1d flip(direction_1d dir) noexcept {
    return static_cast<direction_1d>(to_int(dir) ^ 1);
}

inline int get_adj_level(direction_1d vdir, int level) noexcept {
    return level + 1 - 2 * static_cast<int>(vdir);
}

enum class orientation_2d : uint16_t {
    X = 0,
    Y = 1,
    HORIZONTAL = 0,
    VERTICAL = 1,
    CLOCKWISE = 0,
    COUNTERCLOCKWISE = 1,
};

inline constexpr uint16_t to_int(orientation_2d orient) noexcept {
    return static_cast<int>(orient);
}

inline constexpr orientation_2d perpendicular(orientation_2d orient) noexcept {
    return static_cast<orientation_2d>(to_int(orient) ^ 1);
}

/** Enum of all possible orientation.

    bit 2 = 1 if X/Y axes are swapped.
    bit 1 = 1 if Y coordinates are flipped.
    bit 0 = 1 if X coordinates are flipped.

    axes swapping are done before coordinates flipping.

    NOTE: rotation angle is defined counter-clockwise.  So R90 is rotate left.
 */
enum class orientation : uint16_t {
    NULL_TRANSFORM = 0,
    FLIP_X = 1,
    FLIP_Y = 2,
    FLIP_XY = 3,
    SWAP_XY = 4,
    ROTATE_LEFT = 5,
    ROTATE_RIGHT = 6,
    FLIP_SWAP_XY = 7,
    R0 = 0,
    MY = 1,
    MX = 2,
    R180 = 3,
    MXR90 = 4,
    R90 = 5,
    R270 = 6,
    MYR90 = 7,
};

inline uint16_t to_int(orientation orient) noexcept { return static_cast<int>(orient); }

inline std::string to_string(orientation orient) {
    switch (orient) {
    case orientation::R0:
        return "R0";
    case orientation::R90:
        return "R90";
    case orientation::R180:
        return "R180";
    case orientation::R270:
        return "R270";
    case orientation::MX:
        return "MX";
    case orientation::MY:
        return "MY";
    case orientation::MXR90:
        return "MXR90";
    case orientation::MYR90:
        return "MYR90";
    default:
        throw std::invalid_argument("Unsupported orientation: " +
                                    std::to_string(static_cast<int>(orient)));
    }
}

inline orientation to_orientation(const std::string &val) {
    if (val == "R0")
        return orientation::R0;
    else if (val == "MX")
        return orientation::MX;
    else if (val == "MY")
        return orientation::MY;
    else if (val == "R180")
        return orientation::R180;
    else if (val == "R90")
        return orientation::R90;
    else if (val == "R270")
        return orientation::R270;
    else if (val == "MXR90")
        return orientation::MXR90;
    else if (val == "MYR90")
        return orientation::MYR90;

    throw std::invalid_argument("Unsupported orientation: " + val);
}

inline orientation operator+(orientation a, orientation b) noexcept {
    auto av = static_cast<uint16_t>(a);
    auto bv = static_cast<uint16_t>(b);
    auto a2 = av >> 2;
    auto b2 = bv >> 2;
    auto c2 = a2 ^ b2;
    auto c0 = ((av >> b2) & 1) ^ (bv & 1);
    auto c1 = ((av >> (b2 ^ 1)) & 1) ^ ((bv >> 1) & 1);
    return static_cast<orientation>((c2 << 2) | (c1 << 1) | c0);
}

inline orientation operator-(orientation a) noexcept {
    auto av = static_cast<uint16_t>(a);
    auto a2 = av >> 2;
    auto c0 = (av >> a2) & 1;
    auto c1 = (av >> (a2 ^ 1)) & 1;
    return static_cast<orientation>((a2 << 2) | (c1 << 1) | c0);
}

// returns true if the given rotation/reflection reverses polygon winding direction.
inline bool reverse_winding(orientation orient) noexcept {
    auto b = static_cast<uint16_t>(orient);
    auto b2 = b >> 2;
    auto b1 = (b >> 1) & 1;
    auto b0 = b & 1;
    return b2 ^ b1 ^ b0;
}

// returns true if the given rotation/reflection swaps X and Y axes
inline bool swaps_xy(orientation orient) noexcept {
    return (static_cast<uint16_t>(orient) & 4) != 0;
}

inline std::array<int, 2> axis_scale(orientation orient) noexcept {
    auto code = static_cast<uint16_t>(orient);
    return std::array<int, 2>{1 - 2 * (code & 1), 1 - 2 * ((code >> 1) & 1)};
}

} // namespace cbag

#endif

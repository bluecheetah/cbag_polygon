// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_INTERVAL_DATA_H
#define CBAG_POLYGON_INTERVAL_DATA_H

#include <array>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/interval_concept.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {

template <typename T> class interval_data {
  public:
    using coordinate_type = T;

  private:
    std::array<coordinate_type, 2> coords_;

  public:
    interval_data() noexcept : coords_{0, 0} {};

    interval_data(coordinate_type low, coordinate_type high) noexcept : coords_{low, high} {}

    const coordinate_type &operator[](bool upper) const noexcept { return coords_[upper]; }
    coordinate_type &operator[](bool upper) noexcept { return coords_[upper]; }
    const coordinate_type &operator[](direction_1d dir) const noexcept {
        return coords_[to_int(dir)];
    }
    coordinate_type &operator[](direction_1d dir) noexcept { return coords_[to_int(dir)]; }

    bool operator==(const interval_data &rhs) const noexcept { return coords_ == rhs.coords_; }
};

template <typename T> std::string to_string(const interval_data<T> &obj) {
    std::string ans = "[";
    ans += std::to_string(obj[0]);
    ans += ", ";
    ans += std::to_string(obj[1]);
    ans += ")";
    return ans;
}

template <typename T>
bool operator!=(const interval_data<T> &lhs, const interval_data<T> &rhs) noexcept {
    return !(lhs == rhs);
}

template <typename T> std::ostream &operator<<(std::ostream &stream, const interval_data<T> &obj) {
    stream << to_string(obj);
    return stream;
}

template <typename T> struct tag<interval_data<T>> { using type = interval_tag; };

template <typename T> struct interval_traits<interval_data<T>> {
    using interval_type = interval_data<T>;
    using coordinate_type = typename interval_type::coordinate_type;

    static coordinate_type get(const interval_type &interval, direction_1d dir) noexcept {
        return interval[dir];
    }

    static void set(interval_type &interval, direction_1d dir, coordinate_type value) noexcept {
        interval[dir] = value;
    }

    static interval_type construct(coordinate_type low, coordinate_type high) noexcept {
        return interval_type(low, high);
    }
};

} // namespace polygon
} // namespace cbag

#endif

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POINT_DATA_H
#define CBAG_POLYGON_POINT_DATA_H

#include <array>
#include <ostream>
#include <tuple>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {

template <typename T> class point_data {
  public:
    using coordinate_type = T;

  private:
    std::array<coordinate_type, 2> coords_;

  public:
    point_data() noexcept : coords_{0, 0} {}

    point_data(coordinate_type x, coordinate_type y) noexcept : coords_{x, y} {}

    point_data(orientation_2d orient, coordinate_type c0, coordinate_type c1) noexcept {
        auto val = to_int(orient);
        coords_[val] = c0;
        coords_[1 - val] = c1;
    }

    const coordinate_type &operator[](bool is_y) const noexcept { return coords_[is_y]; }
    coordinate_type &operator[](bool is_y) noexcept { return coords_[is_y]; }
    const coordinate_type &operator[](orientation_2d orient) const noexcept {
        return coords_[to_int(orient)];
    }
    coordinate_type &operator[](orientation_2d orient) noexcept { return coords_[to_int(orient)]; }

    bool operator==(const point_data &rhs) const noexcept { return coords_ == rhs.coords_; }
};

template <typename T> bool operator!=(const point_data<T> &lhs, const point_data<T> &rhs) noexcept {
    return !(lhs == rhs);
}

template <typename T>
point_data<T> operator+(const point_data<T> &lhs, const point_data<T> &rhs) noexcept {
    auto ans = lhs;
    return ans += rhs;
}

template <typename T>
point_data<T> operator-(const point_data<T> &lhs, const point_data<T> &rhs) noexcept {
    auto ans = lhs;
    return ans -= rhs;
}

template <typename T> point_data<T> operator-(const point_data<T> &obj) noexcept {
    return {-obj[0], -obj[1]};
}

template <typename T> std::string to_string(const point_data<T> &obj) {
    std::string ans = "(";
    ans += std::to_string(obj[0]);
    ans += ", ";
    ans += std::to_string(obj[1]);
    ans += ")";
    return ans;
}

template <size_t N, typename T> const T &get(const point_data<T> &obj) noexcept {
    if constexpr (N == 0)
        return obj[0];
    else if constexpr (N == 1)
        return obj[1];
}

template <typename T> T cross(const point_data<T> &v1, const point_data<T> &v2) {
    return v1[0] * v2[1] - v1[1] * v2[0];
}

template <typename T> std::ostream &operator<<(std::ostream &stream, const point_data<T> &obj) {
    stream << to_string(obj);
    return stream;
}

template <typename T> struct tag<point_data<T>> { using type = point_tag; };

template <typename T> struct point_traits<point_data<T>> {
    using point_type = point_data<T>;
    using coordinate_type = typename point_type::coordinate_type;

    static coordinate_type get(const point_type &point, orientation_2d orient) noexcept {
        return point[orient];
    }

    static void set(point_type &point, orientation_2d orient, coordinate_type value) noexcept {
        point[orient] = value;
    }

    static point_type construct(coordinate_type x, coordinate_type y) noexcept {
        return point_type(x, y);
    }
};

} // namespace polygon
} // namespace cbag

// structured binding support
namespace std {

template <typename T>
struct tuple_size<cbag::polygon::point_data<T>> : std::integral_constant<std::size_t, 2> {};

template <size_t N, typename T> struct tuple_element<N, cbag::polygon::point_data<T>> {
    using type = const T;
};

} // namespace std

#endif

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_RECTANGLE_DATA_H
#define CBAG_POLYGON_RECTANGLE_DATA_H

#include <array>
#include <ostream>

#include <cbag/polygon/interval_concept.h>
#include <cbag/polygon/interval_data.h>
#include <cbag/polygon/rectangle_concept.h>

namespace cbag {
namespace polygon {

template <typename T> class rectangle_data {
  public:
    using coordinate_type = T;
    using interval_type = interval_data<coordinate_type>;
    using area_type = typename coordinate_traits<coordinate_type>::area_type;

  private:
    std::array<interval_type, 2> ranges_;

  public:
    rectangle_data() noexcept : ranges_{interval_type{}, interval_type{}} {}
    rectangle_data(T xl, T yl, T xh, T yh) noexcept
        : ranges_{interval_type(xl, xh), interval_type(yl, yh)} {}

    rectangle_data(orientation_2d orient, T tl, T th, T pl, T ph) noexcept {
        auto idx = to_int(orient);
        ranges_[idx] = interval_type(tl, th);
        ranges_[idx ^ 1] = interval_type(pl, ph);
    }

    rectangle_data(const interval_type &intvx, const interval_type &intvy) noexcept
        : ranges_{intvx, intvy} {}

    static rectangle_data get_invalid_bbox() noexcept { return {0, 0, -1, -1}; }

    const interval_type &operator[](bool is_y) const noexcept { return ranges_[is_y]; }
    interval_type &operator[](bool is_y) noexcept { return ranges_[is_y]; }
    const interval_type &operator[](orientation_2d dir) const noexcept {
        return ranges_[to_int(dir)];
    }
    interval_type &operator[](orientation_2d dir) noexcept { return ranges_[to_int(dir)]; }

    bool operator==(const rectangle_data &rhs) const noexcept { return ranges_ == rhs.ranges_; }
};

template <typename T>
bool operator!=(const rectangle_data<T> &lhs, const rectangle_data<T> &rhs) noexcept {
    return !(lhs == rhs);
}

template <typename T> std::string to_string(const rectangle_data<T> &obj) {
    std::string ans = "Box(";
    ans += std::to_string(obj[0][0]);
    ans += ", ";
    ans += std::to_string(obj[1][0]);
    ans += ", ";
    ans += std::to_string(obj[0][1]);
    ans += ", ";
    ans += std::to_string(obj[1][1]);
    ans += ")";
    return ans;
}

template <typename T> std::ostream &operator<<(std::ostream &stream, const rectangle_data<T> &obj) {
    stream << to_string(obj);
    return stream;
}

template <typename T> struct tag<rectangle_data<T>> { using type = rectangle_tag; };

template <typename T> struct rectangle_traits<rectangle_data<T>> {
    using rectangle_type = rectangle_data<T>;
    using interval_type = typename rectangle_type::interval_type;
    using coordinate_type = typename rectangle_type::coordinate_type;
    using area_type = typename rectangle_type::area_type;

    static coordinate_type get_coord(const rectangle_type &rectangle, orientation_2d orient,
                                     direction_1d dir) {
        return rectangle[orient][dir];
    }

    static interval_type get(const rectangle_type &rectangle, orientation_2d orient) {
        return rectangle[orient];
    }

    template <typename Intv, IsInterval<Intv> = 0>
    static void set(rectangle_type &rectangle, orientation_2d orient, const Intv &interval) {
        rectangle[orient][0] = lower(interval);
        rectangle[orient][1] = upper(interval);
    }

    static void set_coord(rectangle_type &rectangle, orientation_2d orient, direction_1d dir,
                          coordinate_type c) {
        rectangle[orient][dir] = c;
    }

    static rectangle_type construct(orientation_2d orient, coordinate_type tl, coordinate_type th,
                                    coordinate_type pl, coordinate_type ph) {
        return {orient, tl, th, pl, ph};
    }
};

} // namespace polygon
} // namespace cbag

#endif

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_RECTANGLE_CONCEPT_H
#define CBAG_POLYGON_RECTANGLE_CONCEPT_H

#include <type_traits>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/math.h>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {

template <typename R> class rect_point_iterator {
  public:
    using rect_traits = get_traits_t<R>;
    using coordinate_type = coord_type<R>;
    using iterator_category = std::forward_iterator_tag;
    using value_type = point_data<coordinate_type>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    std::array<coordinate_type, 4> coords_{0, 0, 0, 0};
    value_type pt_;
    unsigned int idx_ = 4;

  public:
    rect_point_iterator() = default;

    explicit rect_point_iterator(const R &rect) : idx_(0) {
        auto xl = rect_traits::get_coord(rect, orientation_2d::X, direction_1d::LOWER);
        auto xh = rect_traits::get_coord(rect, orientation_2d::X, direction_1d::UPPER);
        auto yl = rect_traits::get_coord(rect, orientation_2d::Y, direction_1d::LOWER);
        auto yh = rect_traits::get_coord(rect, orientation_2d::Y, direction_1d::UPPER);
        coords_[0] = xl;
        coords_[1] = yl;
        coords_[2] = xh;
        coords_[3] = yh;
        pt_[0] = xl;
        pt_[1] = yl;
    }

    bool has_next() const noexcept { return idx_ != 4; }

    bool operator==(const rect_point_iterator &rhs) const noexcept { return idx_ == rhs.idx_; }

    bool operator!=(const rect_point_iterator &rhs) const noexcept { return !(*this == rhs); }

    reference operator*() const noexcept { return pt_; }
    pointer operator->() const noexcept { return &pt_; }

    rect_point_iterator &operator++() {
        pt_[idx_ & 1] = coords_[(idx_ + 2) & 3];
        ++idx_;
        return *this;
    }
};

template <typename R> class rect_compact_iterator {
  public:
    using rect_traits = get_traits_t<R>;
    using coordinate_type = coord_type<R>;
    using iterator_category = std::forward_iterator_tag;
    using value_type = coordinate_type;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    std::array<coordinate_type, 4> coords_{0, 0, 0, 0};
    unsigned int idx_ = 4;

  public:
    rect_compact_iterator() = default;

    rect_compact_iterator(const R &rect) : idx_(0) {
        auto xl = rect_traits::get_coord(rect, orientation_2d::X, direction_1d::LOWER);
        auto xh = rect_traits::get_coord(rect, orientation_2d::X, direction_1d::UPPER);
        auto yl = rect_traits::get_coord(rect, orientation_2d::Y, direction_1d::LOWER);
        auto yh = rect_traits::get_coord(rect, orientation_2d::Y, direction_1d::UPPER);
        coords_[0] = xl;
        coords_[1] = yl;
        coords_[2] = xh;
        coords_[3] = yh;
    }

    bool has_next() const noexcept { return idx_ != 4; }

    bool operator==(const rect_compact_iterator &rhs) const noexcept { return idx_ == rhs.idx_; }

    bool operator!=(const rect_compact_iterator &rhs) const noexcept { return !(*this == rhs); }

    reference operator*() const noexcept { return coords_[idx_]; }
    pointer operator->() const noexcept { return &(coords_[idx_]); }

    rect_compact_iterator &operator++() {
        ++idx_;
        return *this;
    }
};

template <typename T, IsRectangle<T> = 0>
coord_type<T> get_coord(const T &obj, orientation_2d orient, direction_1d dir) {
    return get_traits_t<T>::get_coord(obj, orient, dir);
}

template <typename T, IsRectangle<T> = 0>
typename get_traits_t<T>::interval_type get(const T &obj, orientation_2d orient) {
    return get_traits_t<T>::get(obj, orient);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> lower(const T &obj, orientation_2d orient) {
    return get_coord(obj, orient, direction_1d::LOWER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> upper(const T &obj, orientation_2d orient) {
    return get_coord(obj, orient, direction_1d::UPPER);
}

template <typename T, IsRectangle<T> = 0>
coord_type<T> center(const T &obj, orientation_2d orient) {
    return math::floor2(get_coord(obj, orient, direction_1d::LOWER) +
                        get_coord(obj, orient, direction_1d::UPPER));
}

template <typename T, IsRectangle<T> = 0> coord_type<T> xl(const T &obj) {
    return get_coord(obj, orientation_2d::X, direction_1d::LOWER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> xh(const T &obj) {
    return get_coord(obj, orientation_2d::X, direction_1d::UPPER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> yl(const T &obj) {
    return get_coord(obj, orientation_2d::Y, direction_1d::LOWER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> yh(const T &obj) {
    return get_coord(obj, orientation_2d::Y, direction_1d::UPPER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> xm(const T &obj) {
    return center(obj, orientation_2d::X);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> ym(const T &obj) {
    return center(obj, orientation_2d::Y);
}

template <typename T, IsRectangle<T> = 0>
coord_type<T> dimension(const T &obj, orientation_2d orient) {
    return get_coord(obj, orient, direction_1d::UPPER) -
           get_coord(obj, orient, direction_1d::LOWER);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> width(const T &obj) {
    return dimension(obj, orientation_2d::X);
}

template <typename T, IsRectangle<T> = 0> coord_type<T> height(const T &obj) {
    return dimension(obj, orientation_2d::Y);
}

template <typename T, IsRectangle<T> = 0> point_data<coord_type<T>> point_ll(const T &obj) {
    return {xl(obj), yl(obj)};
}

template <typename T, IsRectangle<T> = 0> point_data<coord_type<T>> point_lr(const T &obj) {
    return {xh(obj), yl(obj)};
}

template <typename T, IsRectangle<T> = 0> point_data<coord_type<T>> point_ul(const T &obj) {
    return {xl(obj), yh(obj)};
}

template <typename T, IsRectangle<T> = 0> point_data<coord_type<T>> point_ur(const T &obj) {
    return {xh(obj), yh(obj)};
}

// return rectangle corners in clockwise order, index 0 is lower-left corner.
template <typename T, IsRectangle<T> = 0>
point_data<coord_type<T>> get_point(const T &obj, int idx) {
    auto ycode = (idx & 2) >> 1;
    auto xdir = static_cast<direction_1d>((idx & 1) ^ ycode);
    auto ydir = static_cast<direction_1d>(ycode);
    return {get_coord(obj, orientation_2d::X, xdir), get_coord(obj, orientation_2d::Y, ydir)};
}

template <typename T, IsRectangle<T> = 0> bool is_physical(const T &obj) {
    return xh(obj) > xl(obj) && yh(obj) > yl(obj);
}

template <typename T, IsRectangle<T> = 0> bool is_valid(const T &obj) {
    return xh(obj) >= xl(obj) && yh(obj) >= yl(obj);
}

template <typename T, IsRectangle<T> = 0>
void set_coord(T &obj, orientation_2d orient, direction_1d dir, coord_type<T> c) {
    get_traits_t<T>::set_coord(obj, orient, dir, c);
}

template <typename T, IsRectangle<T> = 0>
void set_interval(T &obj, orientation_2d orient, coord_type<T> c0, coord_type<T> c1) {
    get_traits_t<T>::set_coord(obj, orient, direction_1d::LOWER, c0);
    get_traits_t<T>::set_coord(obj, orient, direction_1d::UPPER, c1);
}

template <typename T, IsRectangle<T> = 0> T &invert(T &obj) {
    set_interval(obj, orientation_2d::X, xh(obj), xl(obj));
    set_interval(obj, orientation_2d::Y, yh(obj), yl(obj));
    return obj;
}

template <typename T, IsRectangle<T> = 0> T &extend_to(T &obj, coord_type<T> x, coord_type<T> y) {
    if (!is_valid(obj)) {
        set_interval(obj, orientation_2d::X, x, x);
        set_interval(obj, orientation_2d::Y, y, y);
    } else {
        set_interval(obj, orientation_2d::X, std::min(x, xl(obj)), std::max(x, xh(obj)));
        set_interval(obj, orientation_2d::Y, std::min(y, yl(obj)), std::max(y, yh(obj)));
    }
    return obj;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
T1 &operator|=(T1 &lhs, const T2 &rhs) {
    if (is_valid(rhs)) {
        if (is_valid(lhs)) {
            set_interval(lhs, orientation_2d::X, std::min(xl(lhs), xl(rhs)),
                         std::max(xh(lhs), xh(rhs)));
            set_interval(lhs, orientation_2d::Y, std::min(yl(lhs), yl(rhs)),
                         std::max(yh(lhs), yh(rhs)));
        } else {
            set_interval(lhs, orientation_2d::X, xl(rhs), xh(rhs));
            set_interval(lhs, orientation_2d::Y, yl(rhs), yh(rhs));
        }
    }
    return lhs;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
T1 &operator&=(T1 &lhs, const T2 &rhs) {
    if (is_valid(rhs)) {
        if (is_valid(lhs)) {
            set_interval(lhs, orientation_2d::X, std::max(xl(lhs), xl(rhs)),
                         std::min(xh(lhs), xh(rhs)));
            set_interval(lhs, orientation_2d::Y, std::max(yl(lhs), yl(rhs)),
                         std::min(yh(lhs), yh(rhs)));
        } else {
        }
    } else {
        set_interval(lhs, orientation_2d::X, xl(rhs), xh(rhs));
        set_interval(lhs, orientation_2d::Y, yl(rhs), yh(rhs));
    }
    return lhs;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
T1 &operator+=(T1 &lhs, const T2 &rhs) {
    return lhs |= rhs;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
T1 &operator*=(T1 &lhs, const T2 &rhs) {
    return lhs &= rhs;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
T1 &convolve_rect(T1 &lhs, const T2 &rhs) {
    if (is_physical(lhs)) {
        set_coord(lhs, orientation_2d::X, direction_1d::LOWER, xl(lhs) + xl(rhs));
        set_coord(lhs, orientation_2d::Y, direction_1d::LOWER, yl(lhs) + yl(rhs));
        set_coord(lhs, orientation_2d::X, direction_1d::UPPER, xh(lhs) + xh(rhs));
        set_coord(lhs, orientation_2d::Y, direction_1d::UPPER, yh(lhs) + yh(rhs));
    }
    return lhs;
}

template <typename T1, typename T2, IsRectangle<T1> = 0, IsRectangle<T2> = 0>
bool overlaps(const T1 &lhs, const T2 &rhs) {
    auto test = rhs;
    test &= lhs;
    return is_physical(test);
}

template <typename T, IsRectangle<T> = 0>
T &move_by_orient(T &lhs, orientation_2d orient, coord_type<T> dt, coord_type<T> dp) {
    auto perp = perpendicular(orient);
    set_interval(lhs, orient, lower(lhs, orient) + dt, upper(lhs, orient) + dt);
    set_interval(lhs, perp, lower(lhs, perp) + dp, upper(lhs, perp) + dp);
    return lhs;
}

template <typename T, IsRectangle<T> = 0> T &move_by(T &lhs, coord_type<T> dx, coord_type<T> dy) {
    return move_by_orient(lhs, orientation_2d::X, dx, dy);
}

template <typename T, IsRectangle<T> = 0> T get_move_by(T lhs, coord_type<T> dx, coord_type<T> dy) {
    return move_by(lhs, dx, dy);
}

} // namespace polygon
} // namespace cbag

#endif

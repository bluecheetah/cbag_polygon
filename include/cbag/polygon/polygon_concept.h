// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_CONCEPT_H
#define CBAG_POLYGON_POLYGON_CONCEPT_H

#include <vector>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/polygon_45_concept.h>
#include <cbag/polygon/polygon_90_concept.h>
#include <cbag/polygon/rectangle_data.h>
#include <cbag/polygon/tag.h>

namespace cbag {
namespace polygon {
namespace dispatch {

// an iterator that removes duplicate points
template <typename Iter> class poly_reduce_iterator {
  public:
    using coordinate_type = coord_type<typename Iter::value_type>;
    using iterator_category = std::forward_iterator_tag;
    using value_type = point_data<coordinate_type>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    Iter start_, stop_;
    std::array<value_type, 3> pts_;
    unsigned int idx_ = 2;

  public:
    poly_reduce_iterator() = default;

    explicit poly_reduce_iterator(Iter stop) : start_(stop), stop_(stop), idx_(2) {}

    poly_reduce_iterator(Iter start, Iter stop) : start_(start), stop_(stop), idx_(0) {
        pts_[2] = pts_[0] = {x(*start_), y(*start_)};
        ++start_;
        if (start_ == stop_) {
            idx_ = 2;
        } else {
            _get_new_point();
            if (pts_[1] == pts_[0])
                idx_ = 2;
        }
    }

    bool has_next() const noexcept { return start_ != stop_ || idx_ != 2; }

    bool operator==(const poly_reduce_iterator &rhs) const noexcept {
        return start_ == rhs.start_ && idx_ == rhs.idx_;
    }

    bool operator!=(const poly_reduce_iterator &rhs) const noexcept { return !(*this == rhs); }

    reference operator*() const noexcept { return pts_[idx_]; }
    pointer operator->() const noexcept { return &(pts_[idx_]); }

    poly_reduce_iterator &operator++() {
        if (start_ == stop_) {
            if (idx_ == 0) {
                idx_ = (pts_[1] == pts_[0] || pts_[1] == pts_[2]) ? 2 : 1;
            } else {
                idx_ = 2;
            }
        } else {
            pts_[0] = pts_[1];
            _get_new_point();
            if (pts_[1] == pts_[0])
                idx_ = (pts_[1] == pts_[2]) ? 2 : 1;
        }
        return *this;
    }

  private:
    void _get_new_point() {
        pts_[1] = {x(*start_), y(*start_)};
        ++start_;
        while (pts_[1] == pts_[0] && start_ != stop_) {
            pts_[1] = {x(*start_), y(*start_)};
            ++start_;
        }
    }
};

template <typename T> std::size_t size(const T &obj, rectangle_tag) { return 4; }

template <typename T> std::size_t size(const T &obj, polygon_tag) {
    return get_traits_t<T>::size(obj);
}

template <typename T> auto begin_points(const T &obj, rectangle_tag) -> rect_point_iterator<T> {
    return rect_point_iterator<T>(obj);
}

template <typename T>
auto begin_points(const T &obj, polygon_tag) -> typename get_traits_t<T>::iterator_type {
    return get_traits_t<T>::begin_points(obj);
}

template <typename T> auto end_points(const T &obj, rectangle_tag) -> rect_point_iterator<T> {
    return rect_point_iterator<T>();
}

template <typename T>
auto end_points(const T &obj, polygon_tag) -> typename get_traits_t<T>::iterator_type {
    return get_traits_t<T>::end_points(obj);
}

template <typename T> area_type<T> area(const T &obj, rectangle_tag) {
    return static_cast<area_type<T>>(width(obj)) * height(obj);
}

template <typename T> area_type<T> area(const T &obj, polygon_tag) {
    using area_t = area_type<T>;

    auto start = get_traits_t<T>::begin_points(obj);
    auto stop = get_traits_t<T>::end_points(obj);

    if (start == stop)
        return 0;

    auto p0 = point_data<area_t>{x(*start), y(*start)};
    auto p_prev = p0;
    ++start;
    auto sum = static_cast<area_t>(0);
    for (; start != stop; ++start) {
        auto p = point_data<area_t>{x(*start), y(*start)};
        sum += p_prev[0] * p[1] - p[0] * p_prev[1];
        p_prev = p;
    }
    sum += p_prev[0] * p0[1] - p0[0] * p_prev[1];

    return sum / 2;
}

template <typename T> T &expand(T &obj, coord_type<T> dx, coord_type<T> dy, rectangle_tag) {
    set_coord(obj, orientation_2d::X, direction_1d::LOW, xl(obj) - dx);
    set_coord(obj, orientation_2d::X, direction_1d::HIGH, xh(obj) + dx);
    set_coord(obj, orientation_2d::Y, direction_1d::LOW, yl(obj) - dy);
    set_coord(obj, orientation_2d::Y, direction_1d::HIGH, yh(obj) + dy);
    return obj;
}

template <typename T> T &expand(T &obj, coord_type<T> dx, coord_type<T> dy, polygon_90_tag) {
    using coordinate_type = coord_type<T>;

    auto n = size(obj, polygon_90_tag{});
    if (n < 4)
        return obj;

    auto start = get_traits_t<T>::begin_compact(obj);
    auto stop = get_traits_t<T>::end_compact(obj);

    auto iter = poly90_convolve_rect_coord_iterator<decltype(start), coordinate_type>(
        start, stop, -dx, -dy, dx, dy);

    std::vector<coordinate_type> result;
    result.reserve(n);
    for (; iter.has_next(); ++iter) {
        result.emplace_back(*iter);
    }

    get_traits_t<T>::set_compact(obj, std::move(result));
    return obj;
}

template <typename T> T &expand(T &obj, coord_type<T> dx, coord_type<T> dy, polygon_45_tag) {
    using coordinate_type = coord_type<T>;

    auto n = size(obj, polygon_45_tag{});
    if (n < 3)
        return obj;

    auto start = get_traits_t<T>::begin_points(obj);
    auto stop = get_traits_t<T>::end_points(obj);

    auto iter = poly45_convolve_rect_iterator<decltype(start), coordinate_type>(start, stop, -dx,
                                                                                -dy, dx, dy);

    std::vector<point_data<coordinate_type>> result;
    result.reserve(n);
    for (; iter.has_next(); ++iter) {
        result.emplace_back(*iter);
    }

    get_traits_t<T>::set_points(obj, std::move(result));
    return obj;
}

template <typename T> rectangle_data<coord_type<T>> get_bbox(const T &obj, rectangle_tag) {
    return {xl(obj), yl(obj), xh(obj), yh(obj)};
}

template <typename T> rectangle_data<coord_type<T>> get_bbox(const T &obj, polygon_tag) {
    auto ans = rectangle_data<coord_type<T>>::get_invalid_bbox();
    auto stop = end_points(obj, polygon_tag{});
    for (auto iter = begin_points(obj, polygon_tag{}); iter != stop; ++iter) {
        extend_to(ans, x(*iter), y(*iter));
    }
    return ans;
}

template <typename T1, typename T2>
bool contains(const T1 &container, const T2 &obj, rectangle_tag, rectangle_tag) {
    if (!is_valid(container) || !is_valid(obj))
        return false;

    return xl(container) <= xl(obj) && yl(container) <= yl(obj) && xh(obj) <= xh(container) &&
           yh(obj) <= yh(container);
}

template <typename T1, typename T2>
bool contains(const T1 &container, const T2 &obj, rectangle_tag, point_tag) {
    if (!is_valid(container))
        return false;

    auto xp = x(obj);
    auto yp = y(obj);
    return xl(container) <= xp && yl(container) <= yp && xp <= xh(container) && yp <= yh(container);
}

template <typename T1, typename T2>
bool contains(const T1 &container, const T2 &obj, rectangle_tag, polygon_tag) {
    if (!is_valid(container))
        return false;
    auto x0 = xl(container);
    auto y0 = yl(container);
    auto x1 = xh(container);
    auto y1 = yh(container);
    for (const auto &pt : obj) {
        auto xp = x(pt);
        auto yp = y(pt);
        if (x0 > xp || xp > x1 || y0 > yp || yp > y1)
            return false;
    }
    return true;
}

} // namespace dispatch

template <typename T> auto begin_points(const T &obj) {
    return dispatch::begin_points(obj, typename tag<T>::type{});
}

template <typename T> auto end_points(const T &obj) {
    return dispatch::end_points(obj, typename tag<T>::type{});
}

template <typename T> area_type<T> area(const T &obj) {
    return dispatch::area(obj, typename tag<T>::type{});
}

template <typename T> bool is_positive(const T &obj) { return area(obj) > 0; }

template <typename T> std::size_t size(const T &obj) {
    return dispatch::size(obj, typename tag<T>::type{});
}

template <typename T> void set_points(T &obj, std::vector<typename T::point_type> &&data) {
    return get_traits_t<T>::set_points(obj, std::move(data));
}

template <typename T> T &expand(T &obj, coord_type<T> dx, coord_type<T> dy) {
    return dispatch::expand(obj, dx, dy, typename tag<T>::type{});
}

template <typename T> T get_expand(const T &obj, coord_type<T> dx, coord_type<T> dy) {
    auto ans = obj;
    return dispatch::expand(ans, dx, dy, typename tag<T>::type{});
}

template <typename T> rectangle_data<coord_type<T>> get_bbox(const T &obj) {
    return dispatch::get_bbox(obj, typename tag<T>::type{});
}

} // namespace polygon
} // namespace cbag

#endif

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_90_CONCEPT_H
#define CBAG_POLYGON_POLYGON_90_CONCEPT_H

#include <array>
#include <cbag/polygon/rectangle_data.h>

namespace cbag {
namespace polygon {

template <typename Iter, typename T> class poly90_point_iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = point_data<T>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    Iter begin_;
    Iter end_;
    Iter cur_;
    value_type val_;
    bool cur_idx_ = true;

  public:
    poly90_point_iterator() = default;

    explicit poly90_point_iterator(Iter end) : end_(end), cur_(end) {}

    poly90_point_iterator(Iter begin, Iter end) : begin_(begin), end_(end), cur_(begin) {
        if (cur_ != end_) {
            val_[0] = *cur_;
            ++cur_;
            if (cur_ != end_)
                val_[1] = *cur_;
        }
    }

    auto operator++() -> poly90_point_iterator & {
        if (cur_ != end_)
            ++cur_;
        cur_idx_ ^= 1;
        if (cur_ != end_) {
            val_[cur_idx_] = *cur_;
        } else {
            val_[cur_idx_] = *begin_;
        }
        return *this;
    }
    auto operator*() const noexcept -> reference { return val_; }
    auto operator-> () const noexcept -> pointer { return &val_; }
    bool has_next() const noexcept { return cur_ != end_ || cur_idx_ != true; }
    bool operator==(const poly90_point_iterator &rhs) const noexcept {
        return cur_ == rhs.cur_ && cur_idx_ == rhs.cur_idx_;
    }
    bool operator!=(const poly90_point_iterator &rhs) const noexcept { return !(*this == rhs); }
};

// precondition: there's at least 4 coordinates
// NOTE: fails on duplicate vertices
template <typename Iter, typename T> class poly90_convolve_rect_coord_iterator {
  public:
    using coordinate_type = T;
    using iterator_category = std::forward_iterator_tag;
    using value_type = coordinate_type;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    Iter iter_, stop_;
    rectangle_data<coordinate_type> rect_;
    coordinate_type val_ = 0;
    std::array<coordinate_type, 6> coords_ = {0, 0, 0, 0, 0, 0};
    bool orient_idx_ = false;
    int idx_ = 5;

  public:
    poly90_convolve_rect_coord_iterator() = default;

    explicit poly90_convolve_rect_coord_iterator(Iter stop) : iter_(stop), idx_(5) {}

    poly90_convolve_rect_coord_iterator(Iter start, Iter stop, T xl, T yl, T xh, T yh)
        : iter_(start), stop_(stop), rect_(xl, yl, xh, yh), idx_(1) {
        coords_[3] = *iter_;
        ++iter_;
        coords_[0] = coords_[4] = *iter_;
        ++iter_;
        coords_[1] = coords_[5] = *iter_;
        ++iter_;
        coords_[2] = *iter_;
        ++iter_;
        _update_val();
    }

    auto operator++() -> poly90_convolve_rect_coord_iterator & {
        orient_idx_ ^= 1;
        if (iter_ == stop_) {
            ++idx_;
        } else {
            coords_[0] = coords_[1];
            coords_[1] = coords_[2];
            coords_[2] = *iter_;
            ++iter_;
        }
        if (idx_ < 5) {
            _update_val();
        }
        return *this;
    }
    auto operator*() const noexcept -> reference { return val_; }
    auto operator-> () const noexcept -> pointer { return &val_; }
    bool has_next() const noexcept { return iter_ != stop_ || idx_ != 5; }
    bool operator==(const poly90_convolve_rect_coord_iterator &rhs) const noexcept {
        return iter_ == rhs.iter_ && idx_ == rhs.idx_;
    }
    bool operator!=(const poly90_convolve_rect_coord_iterator &rhs) const noexcept {
        return !(*this == rhs);
    }

  private:
    void _update_val() {
        if (coords_[idx_ + 1] == coords_[idx_ - 1])
            throw std::runtime_error("poly90_convolve_rect_coord_iterator cannot "
                                     "process duplicate vertices.");
        auto delta = (coords_[idx_ + 1] - coords_[idx_ - 1]) > 0;
        val_ = coords_[idx_] + rect_[orient_idx_][orient_idx_ ^ delta];
    }
};

namespace dispatch {

template <typename T> rect_compact_iterator<T> begin_compact(const T &obj, rectangle_tag) {
    return rect_compact_iterator<T>(obj);
}

template <typename T> rect_compact_iterator<T> end_compact(const T &obj, rectangle_tag) {
    return rect_compact_iterator<T>();
}

template <typename T>
typename get_traits_t<T>::compact_iterator_type begin_compact(const T &obj, polygon_90_tag) {
    return get_traits_t<T>::begin_compact(obj);
}

template <typename T>
typename get_traits_t<T>::compact_iterator_type end_compact(const T &obj, polygon_90_tag) {
    return get_traits_t<T>::end_compact(obj);
}

} // namespace dispatch

template <typename T> auto begin_compact(const T &obj) {
    return dispatch::begin_compact(obj, typename tag<T>::type{});
}

template <typename T> auto end_compact(const T &obj) {
    return dispatch::end_compact(obj, typename tag<T>::type{});
}

} // namespace polygon
} // namespace cbag

#endif

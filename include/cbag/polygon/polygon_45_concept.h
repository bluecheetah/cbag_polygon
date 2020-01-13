// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_45_CONCEPT_H
#define CBAG_POLYGON_POLYGON_45_CONCEPT_H

#include <array>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/rectangle_data.h>

namespace cbag {
namespace polygon {

/*
  TODO: fix poly45_convolve_rect implementation.
  Right now this iterator generates self-intersecting polygon if
  polygon has concave boundaries (which is OK if you add it to polygon_set_data),
  and it flat out fails for negative polygons.

  To fix this, algorithm need to be changed to the following:

  1. at eaceh vertex, compute the corresponding rectangle vertex for the incoming
     and outgoing edges.

  2. if the rectangle vertices are the same, just output it.

  3. if they are different, the old implementation walks along the rectangle in
     counter-clockwise direction to connect them.  Instead, you should
     convert the incoming/outgoing edges into semi-infinite lines starting/ending
     at the rectangle vertices.  If these two semi-infinite lines intersect,
     output the intersection.  Otherwise, join the two rectangle vertices
     using the rectangle boundary.
 */

// precondition: there's at least 3 points
// NOTE: fails on duplicate vertices
template <typename Iter, typename T> class poly45_convolve_rect_iterator {
  public:
    using coordinate_type = T;
    using iterator_category = std::forward_iterator_tag;
    using value_type = point_data<coordinate_type>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = const value_type &;

  private:
    Iter iter_, stop_;
    rectangle_data<coordinate_type> rect_;
    std::array<value_type, 5> pts_;
    value_type val_;
    int pt_idx_ = 4;
    int8_t rpt0_ = 0;
    int8_t rpt1_ = 0;

  public:
    poly45_convolve_rect_iterator() = default;

    explicit poly45_convolve_rect_iterator(Iter stop) : iter_(stop) {}

    poly45_convolve_rect_iterator(Iter start, Iter stop, T xl, T yl, T xh, T yh)
        : iter_(start), stop_(stop), rect_(xl, yl, xh, yh), pt_idx_(1) {
        pts_[0] = pts_[3] = {x(*iter_), y(*iter_)};
        ++iter_;
        pts_[1] = pts_[4] = {x(*iter_), y(*iter_)};
        if (pts_[0] == pts_[1])
            throw std::runtime_error("poly45_convolve_rect_iterator cannot process "
                                     "duplicate vertices.");
        ++iter_;
        pts_[2] = {x(*iter_), y(*iter_)};
        ++iter_;
        _update_val();
    }

    auto operator++() -> poly45_convolve_rect_iterator & {
        if (rpt0_ != rpt1_) {
            rpt0_ = (rpt0_ + 1) & 3;
            val_ = pts_[pt_idx_] + get_point(rect_, rpt0_);
        } else {
            if (iter_ == stop_) {
                ++pt_idx_;
            } else {
                pts_[0] = pts_[1];
                pts_[1] = pts_[2];
                pts_[2] = {x(*iter_), y(*iter_)};
                ++iter_;
            }
            if (pt_idx_ < 4) {
                _update_val();
            } else {
                rpt0_ = rpt1_ = 0;
            }
        }
        return *this;
    }
    auto operator*() const noexcept -> reference { return val_; }
    auto operator-> () const noexcept -> pointer { return &val_; }
    bool has_next() const noexcept { return pt_idx_ != 4; }
    bool operator==(const poly45_convolve_rect_iterator &rhs) const noexcept {
        return iter_ == rhs.iter_ && pt_idx_ == rhs.pt_idx_ && rpt0_ == rhs.rpt0_ &&
               rpt1_ == rhs.rpt1_;
    }
    bool operator!=(const poly45_convolve_rect_iterator &rhs) const noexcept {
        return !(*this == rhs);
    }

  private:
    void _update_val() {
        if (pts_[pt_idx_ + 1] == pts_[pt_idx_])
            throw std::runtime_error("poly45_convolve_rect_iterator cannot process "
                                     "duplicate vertices.");
        auto dx0 = pts_[pt_idx_][0] - pts_[pt_idx_ - 1][0];
        auto dy0 = pts_[pt_idx_][1] - pts_[pt_idx_ - 1][1];
        auto dx1 = pts_[pt_idx_ + 1][0] - pts_[pt_idx_][0];
        auto dy1 = pts_[pt_idx_ + 1][1] - pts_[pt_idx_][1];

        rpt0_ = _get_rect_corner_index(dx0, dy0, true);
        rpt1_ = _get_rect_corner_index(dx1, dy1, false);
        val_ = pts_[pt_idx_] + get_point(rect_, rpt0_);
    }

    int _get_rect_corner_index(coordinate_type dx, coordinate_type dy, bool in) {
        auto ans = ((dy != 0 && dx == 0) || (dx == dy)) | ((dx < 0 || (dx == 0 && dy < 0)) << 1);
        return (ans + (in && (dy == 0 || dx == 0))) & 3;
    }
};

} // namespace polygon
} // namespace cbag

#endif

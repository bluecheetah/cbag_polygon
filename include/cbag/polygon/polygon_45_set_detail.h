// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_45_SET_DETAIL_H
#define CBAG_POLYGON_POLYGON_45_SET_DETAIL_H

#include <array>
#include <bitset>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include <fmt/core.h>

#include <cbag/polygon/math.h>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/transformation.h>

namespace cbag {
namespace polygon {
namespace detail_poly45 {

template <class Derived> class binary_cnt {
  private:
    std::array<int, 2> val_{0, 0};

  public:
    binary_cnt() = default;

    binary_cnt(int val) noexcept : val_{val, val} {}

    int operator[](bool is_right) const noexcept { return val_[is_right]; }

    void increment(bool is_right, int val) noexcept { val_[is_right] += val; }

    Derived &operator=(int val) noexcept {
        val_[0] = val_[1] = val;
        return static_cast<Derived &>(*this);
    }
};

template <typename D> bool operator==(const binary_cnt<D> &lhs, int rhs) noexcept {
    return lhs[0] == rhs && lhs[1] == rhs;
}

template <typename D> bool operator!=(const binary_cnt<D> &lhs, int rhs) noexcept {
    return !(lhs == rhs);
}

template <typename D> D &operator+=(binary_cnt<D> &lhs, const D &rhs) noexcept {
    lhs.increment(0, rhs[0]);
    lhs.increment(1, rhs[1]);
    return static_cast<D &>(lhs);
}

template <typename D> D &operator-=(binary_cnt<D> &lhs, const D &rhs) noexcept {
    lhs.increment(0, -rhs[0]);
    lhs.increment(1, -rhs[1]);
    return static_cast<D &>(lhs);
}

template <typename D> D operator+(const binary_cnt<D> &lhs, const D &rhs) noexcept {
    auto ans = D(static_cast<const D &>(lhs));
    return ans += rhs;
}

template <typename D> D operator-(const binary_cnt<D> &lhs, const D &rhs) noexcept {
    auto ans = D(static_cast<const D &>(lhs));
    return ans -= rhs;
}

template <typename D> std::ostream &operator<<(std::ostream &stream, const binary_cnt<D> &rhs) {
    stream << fmt::format("[{}, {}]", rhs[0], rhs[1]);
    return stream;
}

class and_cnt : public binary_cnt<and_cnt> {
  public:
    using base_type = binary_cnt<and_cnt>;
    using base_type::base_type;
};

class minus_cnt : public binary_cnt<minus_cnt> {
  public:
    using base_type = binary_cnt<minus_cnt>;
    using base_type::base_type;
};

inline bool operator>(const and_cnt &lhs, int rhs) noexcept { return lhs[0] > rhs && lhs[1] > rhs; }

inline bool operator>(const minus_cnt &lhs, int rhs) noexcept {
    return lhs[0] > rhs && !(lhs[1] > rhs);
}

template <typename CntType> class edge_cnt {
  private:
    std::array<CntType, 3> cnt_arr_{0, 0, 0};

  public:
    edge_cnt() = default;

    edge_cnt(int val) noexcept : cnt_arr_{val, val, val} {}

    edge_cnt &operator=(int val) noexcept {
        cnt_arr_[0] = val;
        cnt_arr_[1] = val;
        cnt_arr_[2] = val;
        return *this;
    }

    CntType count(int idx) const { return cnt_arr_[idx]; }

    CntType edge_type_count(int8_t edge_type) const { return cnt_arr_[edge_type + 1]; }

    void reset() noexcept { cnt_arr_[0] = cnt_arr_[1] = cnt_arr_[2] = 0; }

    edge_cnt &operator+=(const edge_cnt &rhs) noexcept {
        cnt_arr_[0] += rhs.cnt_arr_[0];
        cnt_arr_[1] += rhs.cnt_arr_[1];
        cnt_arr_[2] += rhs.cnt_arr_[2];
        return *this;
    }

    void add_count(bool is_right, const edge_cnt<int> &delta) noexcept {
        cnt_arr_[0].increment(is_right, delta.count(0));
        cnt_arr_[1].increment(is_right, delta.count(1));
        cnt_arr_[2].increment(is_right, delta.count(2));
    }

    void add_edge(int8_t edge_type, CntType val) { cnt_arr_[edge_type + 1] += val; }
};

template <typename CntType>
std::ostream &operator<<(std::ostream &stream, const edge_cnt<CntType> &obj) {
    stream << fmt::format("({}, {}, {})", obj.count(0), obj.count(1), obj.count(2));
    return stream;
}

template <typename T, orientation_2d Orient = orientation_2d::X> class polygon_45_set_vertex {
  public:
    using coordinate_type = T;
    using point_type = point_data<coordinate_type>;

  private:
    point_type pt_;
    uint8_t edge_type_ = 0;
    uint8_t edge_in_ = 0;

  public:
    polygon_45_set_vertex() = default;

    polygon_45_set_vertex(coordinate_type x, coordinate_type y, uint8_t edge_type, uint8_t edge_in)
        : pt_(x, y), edge_type_(edge_type), edge_in_(edge_in) {}

    polygon_45_set_vertex(coordinate_type pos, coordinate_type coord, bool is_up, int8_t edge_type,
                          bool in_edge) noexcept {
        pt_[Orient] = coord;
        pt_[perpendicular(Orient)] = pos;
        add_edge(is_up, edge_type, in_edge);
    }

    polygon_45_set_vertex(coordinate_type c0, coordinate_type c1, coordinate_type c2,
                          coordinate_type c3, bool x_first, bool invert) noexcept {
        if (x_first) {
            pt_[0] = c2;
            pt_[1] = c1;
            _edge_from_delta(math::sign_diff(c0, c2), 0, !invert);
            _edge_from_delta(0, math::sign_diff(c3, c1), invert);
        } else {
            pt_[0] = c1;
            pt_[1] = c2;
            _edge_from_delta(0, math::sign_diff(c0, c2), !invert);
            _edge_from_delta(math::sign_diff(c3, c1), 0, invert);
        }
    }

    polygon_45_set_vertex(coordinate_type x0, coordinate_type y0, coordinate_type x1,
                          coordinate_type y1, coordinate_type x2, coordinate_type y2, bool invert)
        : pt_(x1, y1) {
        auto dx0 = x0 - x1;
        auto dx2 = x2 - x1;
        auto dy0 = y0 - y1;
        auto dy2 = y2 - y1;

        if ((dx0 != 0 && dy0 != 0 && std::abs(dx0) != std::abs(dy0)) ||
            (dx2 != 0 && dy2 != 0 && std::abs(dx2) != std::abs(dy2))) {
            auto msg =
                fmt::format("points [({}, {}), ({}, {}), ({}, {})] form non-45 degree angle.", x0,
                            y0, x1, y1, x2, y2);
            throw std::invalid_argument(msg);
        };

        _edge_from_delta(dx0, dy0, !invert);
        _edge_from_delta(dx2, dy2, invert);
    }

    bool operator==(const polygon_45_set_vertex &rhs) const noexcept {
        return pt_ == rhs.pt_ && edge_type_ == rhs.edge_type_ && edge_in_ == rhs.edge_in_;
    }

    bool operator<(const polygon_45_set_vertex &rhs) const noexcept {
        auto p0 = pos();
        auto p1 = rhs.pos();
        auto c0 = coord();
        auto c1 = rhs.coord();
        return p0 < p1 || (p0 == p1 && (c0 < c1 || (c0 == c1 && (edge_type_ < rhs.edge_type_ ||
                                                                 (edge_type_ == rhs.edge_type_ &&
                                                                  edge_in_ < rhs.edge_in_)))));
    }

    coordinate_type pos() const noexcept { return pt_[perpendicular(Orient)]; }

    coordinate_type coord() const noexcept { return pt_[Orient]; }

    coordinate_type x() const noexcept { return pt_[0]; }

    coordinate_type y() const noexcept { return pt_[1]; }

    bool has_diagonal() const noexcept { return (edge_type_ & 0xAA) != 0; }

    edge_cnt<int> edge_info(bool is_up) const noexcept {
        auto ans = edge_cnt<int>{};
        auto cur_edge = -1;
        auto mask = 1 << (2 + cur_edge + 4 * is_up);
        ans.add_edge(cur_edge, ((edge_type_ & mask) != 0) * (2 * ((edge_in_ & mask) != 0) - 1));
        ++cur_edge;
        mask <<= 1;
        ans.add_edge(cur_edge, ((edge_type_ & mask) != 0) * (2 * ((edge_in_ & mask) != 0) - 1));
        ++cur_edge;
        mask <<= 1;
        ans.add_edge(cur_edge, ((edge_type_ & mask) != 0) * (2 * ((edge_in_ & mask) != 0) - 1));
        return ans;
    }

    int vertical_edge_count(bool is_up) const noexcept {
        auto mask = 1 << (2 + 4 * is_up);
        return ((edge_type_ & mask) != 0) * (2 * ((edge_in_ & mask) != 0) - 1);
    }

    std::array<bool, 2> get_vertical_up_info() const noexcept {
        auto tmp = 1 << 6;
        return std::array<bool, 2>{(edge_type_ & tmp) != 0, (edge_in_ & tmp) != 0};
    }

    std::tuple<std::size_t, std::array<int8_t, 3>, std::array<bool, 3>> up_edge_list() const
        noexcept {
        auto ans = std::make_tuple(static_cast<std::size_t>(0), std::array<int8_t, 3>{0, 0, 0},
                                   std::array<bool, 3>{false, false, false});
        auto num = 0;
        if (edge_type_ & (1 << 5)) {
            std::get<1>(ans)[num] = -1;
            std::get<2>(ans)[num] = (edge_in_ & (1 << 5)) != 0;
            ++num;
        }
        if (edge_type_ & (1 << 6)) {
            std::get<1>(ans)[num] = 0;
            std::get<2>(ans)[num] = (edge_in_ & (1 << 6)) != 0;
            ++num;
        }
        if (edge_type_ & (1 << 7)) {
            std::get<1>(ans)[num] = 1;
            std::get<2>(ans)[num] = (edge_in_ & (1 << 7)) != 0;
            ++num;
        }
        std::get<0>(ans) = num;
        return ans;
    }

    std::string to_string() const {
        return fmt::format("({}, {}, {}, {})", x(), y(), std::bitset<8>(edge_type_).to_string(),
                           std::bitset<8>(edge_in_).to_string());
    }

    void move_by(coordinate_type dx = 0, coordinate_type dy = 0) noexcept {
        pt_[0] += dx;
        pt_[1] += dy;
    }

    void transform(const transformation<coordinate_type> &xform) noexcept {
        // transform location
        pt_ = xform.transform(pt_[0], pt_[1]);

        // transform flags
        auto ocode = static_cast<unsigned int>(xform.orient());
        if (ocode & 4) {
            // swap axis
            edge_type_ = math::rotr8(math::reverse_bits(edge_type_), 1);
            edge_in_ = math::rotr8(math::reverse_bits(edge_in_), 1);
        }
        if (ocode & 2) {
            // flip Y coordinates
            edge_type_ = math::rotl8(math::reverse_bits(edge_type_), 1);
            edge_in_ = math::rotl8(math::reverse_bits(edge_in_), 1);
        }
        if (ocode & 1) {
            // flip X coordinates
            edge_type_ = math::rotr8(math::reverse_bits(edge_type_), 3);
            edge_in_ = math::rotr8(math::reverse_bits(edge_in_), 3);
        }
        if (reverse_winding(xform.orient())) {
            edge_in_ ^= edge_type_;
        }
    }

    void invert() noexcept {
        edge_in_ ^= 0xff;
        edge_in_ &= edge_type_;
    }

    void add_edge(bool is_up, int8_t edge_type, bool in_edge) noexcept {
        auto tmp = 1 << (2 + edge_type + 4 * is_up);
        edge_type_ |= tmp;
        edge_in_ |= tmp * in_edge;
    }

    void add_h_edge(bool is_right, bool is_in) {
        auto tmp = 1 << (4 * (!is_right));
        edge_type_ |= tmp;
        edge_in_ |= tmp * is_in;
    }

  private:
    void _edge_from_delta(coordinate_type dx, coordinate_type dy, bool is_in) noexcept {
        auto tmp = (dy > 0 && dx > 0) << 7 | (dy > 0 && dx == 0) << 6 | (dy > 0 && dx < 0) << 5 |
                   (dy == 0 && dx < 0) << 4 | (dy < 0 && dx < 0) << 3 | (dy < 0 && dx == 0) << 2 |
                   (dy < 0 && dx > 0) << 1 | (dy == 0 && dx > 0);
        // NOTE: use XOR so that two edges with opposite directions cancel out.
        // this method is guaranteed to be called exactly twice, with different is_in values.
        edge_type_ ^= tmp;
        edge_in_ ^= tmp * is_in;
    }
};

template <typename T, orientation_2d O>
bool operator!=(const polygon_45_set_vertex<T, O> &lhs,
                const polygon_45_set_vertex<T, O> &rhs) noexcept {
    return !(lhs == rhs);
}

template <typename T, orientation_2d O>
bool operator>(const polygon_45_set_vertex<T, O> &lhs,
               const polygon_45_set_vertex<T, O> &rhs) noexcept {
    return rhs < lhs;
}

template <typename T, orientation_2d O>
bool operator<=(const polygon_45_set_vertex<T, O> &lhs,
                const polygon_45_set_vertex<T, O> &rhs) noexcept {
    return !(rhs < lhs);
}

template <typename T, orientation_2d O>
bool operator>=(const polygon_45_set_vertex<T, O> &lhs,
                const polygon_45_set_vertex<T, O> &rhs) noexcept {
    return !(lhs < rhs);
}

template <typename T>
std::ostream &operator<<(std::ostream &stream, const polygon_45_set_vertex<T> &obj) {
    stream << obj.to_string();
    return stream;
}

template <typename T> class intersect_vertex {
  public:
    using coordinate_type = T;

  private:
    coordinate_type pos_ = 0;
    coordinate_type coord_ = 0;
    bool invalid_ = false;

  public:
    intersect_vertex() = default;

    intersect_vertex(coordinate_type pos, coordinate_type coord, bool invalid) noexcept
        : pos_(pos), coord_(coord), invalid_(invalid) {}

    bool operator<(const intersect_vertex &rhs) const noexcept {
        return pos_ < rhs.pos_ ||
               (pos_ == rhs.pos_ &&
                (coord_ < rhs.coord_ || (coord_ == rhs.coord_ && invalid_ < rhs.invalid_)));
    }

    coordinate_type pos() const noexcept { return pos_; }
    coordinate_type coord() const noexcept { return coord_; }
    bool invalid() const noexcept { return invalid_; }
};

template <typename T>
bool operator>(const intersect_vertex<T> &lhs, const intersect_vertex<T> &rhs) noexcept {
    return rhs < lhs;
}

template <typename T> class poly_vertex {
  public:
    using coordinate_type = T;

  private:
    coordinate_type pos_ = 0;
    coordinate_type coord_ = 0;
    int8_t edge_type_ = 0;

  public:
    poly_vertex() = default;

    poly_vertex(coordinate_type pos, coordinate_type coord, int8_t edge_type) noexcept
        : pos_(pos), coord_(coord), edge_type_(edge_type) {}

    coordinate_type coord(coordinate_type pos) const noexcept {
        return (pos - pos_) * edge_type_ + coord_;
    }

    int8_t edge_type() const noexcept { return edge_type_; }

    bool operator<(const poly_vertex &rhs) const noexcept {
        auto max_pos = std::max(pos_, rhs.pos_);
        auto coord_l = coord(max_pos);
        auto coord_r = rhs.coord(max_pos);
        return (pos_ == rhs.pos_ &&
                (coord_ < rhs.coord_ || (coord_ == rhs.coord_ && edge_type_ < rhs.edge_type_))) ||
               (pos_ != rhs.pos_ &&
                (coord_l < coord_r || (coord_l == coord_r && rhs.edge_type_ < edge_type_)));
    }

    // precondition: at pos, this vertex is on the left of rhs
    std::optional<intersect_vertex<coordinate_type>>
    get_intersection(coordinate_type pos, const poly_vertex &rhs) const {
        auto ans = std::optional<intersect_vertex<coordinate_type>>();

        if (edge_type_ == rhs.edge_type_) {
            return ans;
        }

        auto numer = rhs.coord_ - coord_ + edge_type_ * pos_ - rhs.edge_type_ * rhs.pos_;
        auto denom = edge_type_ - rhs.edge_type_;

        auto pos_intersect = numer / denom;
        auto pos_remainder = numer % denom;
        auto invalid = (pos_remainder != 0);

        // pos_intersect is always less than or equal to the real position
        // in case where intersection is not on grid, because integer
        // division discards fraction, if exactly one number is negative,
        // we need to decrease pos_intersect by 1
        if (invalid && ((numer < 0) ^ (denom < 0))) {
            --pos_intersect;
        }

        // we evaluate the coordinate with this vertex.  In this case,
        // the true intersection point is always to the upper-right of
        // the recorded intersection point
        if (pos_intersect > pos || (pos_intersect == pos && invalid)) {
            ans.emplace(pos_intersect, coord(pos_intersect), invalid);
        }

        return ans;
    }

    void set(coordinate_type pos, coordinate_type coord, int8_t edge) noexcept {
        pos_ = pos;
        coord_ = coord;
        edge_type_ = edge;
    }
};

template <typename T, typename OutIter, typename ValueType, orientation_2d Orient> class get_poly {
  public:
    using coordinate_type = T;
    using vertex_type = poly_vertex<coordinate_type>;

    struct poly_val {
      public:
        bool is_head = false;
        std::size_t poly_id = 0;
    };

    struct poly_info {
      public:
        std::deque<point_data<coordinate_type>> points;

        poly_info() = default;

        coordinate_type get_bound(bool upper) const {
            auto idx = (points.size() - 1) * upper;
            return points[idx][Orient];
        }
    };

    using intv_map_type = std::map<vertex_type, poly_val>;
    using intv_map_iterator = typename intv_map_type::iterator;

  private:
    OutIter output_;
    intv_map_type intv_map_;
    intv_map_iterator iter_;
    intv_map_iterator open_poly_;
    std::vector<poly_info> poly_list_;
    coordinate_type pos_ = 0;
    bool fracture_ = true;

  public:
    get_poly(OutIter output, bool fracture)
        : output_(output), iter_(intv_map_.end()), open_poly_(iter_), fracture_(fracture) {}

    void process_vertex(const polygon_45_set_vertex<coordinate_type, Orient> &vertex) {
        auto coord = vertex.coord();
        auto[num_up, edge_arr, cnt_arr] = vertex.up_edge_list();
        auto up_idx = 0;

        _update_pos(vertex.pos());
        auto num_dn = _update_coord(coord);

        // try to complete polygons first
        if (num_dn > 1) {
            auto next_dn_iter = std::next(iter_, 1);
            if (iter_->second.poly_id == next_dn_iter->second.poly_id) {
                iter_ = _join_poly(iter_, next_dn_iter);
                if (iter_->first.coord(pos_) != coord) {
                    // all edges are processed
                    // this check is here because in the case of completing
                    // negative polygon and fracturing to the same coordinate,
                    // all three edges could be processed
                    num_dn = 0;
                } else {
                    num_dn -= 2;
                }
            } else if (num_dn == 3) {
                auto third_iter = std::next(next_dn_iter, 1);
                if (next_dn_iter->second.poly_id == third_iter->second.poly_id) {
                    _join_poly(next_dn_iter, third_iter);
                    num_dn -= 2;
                }
            }
        }

        // process current open polygon
        auto map_end = intv_map_.end();
        if (open_poly_ != map_end) {
            if (num_dn > 0) {
                // connect with lowest edge
                auto tmp = open_poly_;
                open_poly_ = map_end;
                iter_ = _join_poly(tmp, iter_);
                --num_dn;
            } else if (num_up > 0) {
                // the current vertex is new, as num_dn == 0
                auto pid = open_poly_->second.poly_id;
                auto prev_is_head = open_poly_->second.is_head;
                if (poly_list_[pid].points.size() == 1) {
                    // adding the second point of poly interval
                    _add_new_intv_point(pid, coord, edge_arr[up_idx], !prev_is_head);
                } else {
                    // appending to an existing polygon
                    _append_point(pid, coord, prev_is_head);
                    // update the last node
                    auto node = intv_map_.extract(open_poly_);
                    node.key().set(pos_, coord, edge_arr[up_idx]);
                    intv_map_.insert(iter_, std::move(node));
                }
                open_poly_ = map_end;
                ++up_idx;
                --num_up;
            }
            // in cases where we have a hole vertex abut the boundary,
            // we could have an open poly but no up/down edges.
            // in this case, this vertex should be skipped
        }

        // process down edges first
        if (num_dn > 1) {
            // 2 or more down edges, join the corresponding polygons
            // prioritize joining edges that form positive polygons
            auto next_dn_iter = std::next(iter_, 1);
            if (num_dn == 3 && !(iter_->second.is_head)) {
                _join_poly(next_dn_iter, std::next(next_dn_iter, 1));
            } else {
                iter_ = _join_poly(iter_, next_dn_iter);
            }
            num_dn -= 2;
        }
        if (num_dn == 1) {
            // add current point to poly
            _append_point(iter_->second.poly_id, coord, iter_->second.is_head);
            if ((num_up & 1) == 0) {
                // even up edges, so we must have a horizontal edge
                // prefer grouping down edge with horizontal edge
                open_poly_ = iter_;
            } else {
                // either 1 or 3 up edges.  join with highest up edge by default,
                // so we need to modify edge direction
                auto edge_up = edge_arr[up_idx + num_up - 1];
                auto node = intv_map_.extract(iter_++);
                node.key().set(pos_, coord, edge_up);
                intv_map_.insert(iter_, std::move(node));
                --num_up;
            }
        }

        if (num_up > 1) {
            // add new polygon with point at bottom
            if (num_up == 3 && !cnt_arr[up_idx]) {
                _new_poly(coord, edge_arr[up_idx + 1], edge_arr[up_idx + 2], cnt_arr[up_idx + 1]);
            } else {
                _new_poly(coord, edge_arr[up_idx], edge_arr[up_idx + 1], cnt_arr[up_idx]);
                up_idx += 2;
            }
            num_up -= 2;
        }
        if (num_up == 1) {
            // add new open polygon
            auto pid = poly_list_.size();
            poly_list_.emplace_back();
            open_poly_ = _add_new_intv_point(pid, coord, edge_arr[up_idx], cnt_arr[up_idx]);
        }
    }

  private:
    void _update_pos(coordinate_type pos) {
        if (pos_ != pos) {
            pos_ = pos;
            iter_ = intv_map_.begin();
        }
    }

    std::size_t _update_coord(coordinate_type coord) {
        auto stop = intv_map_.end();
        if (iter_ == stop)
            return 0;

        if (iter_->first.coord(pos_) < coord) {
            iter_ = intv_map_.lower_bound(vertex_type{pos_, coord, 1});
        }

        auto ans = 0;
        for (auto test = iter_; test != stop && test->first.coord(pos_) == coord; ++test) {
            ++ans;
        }
        return ans;
    }

    intv_map_iterator _find_other_poly(coordinate_type coord, std::size_t poly_id) {
        auto iter_bot = intv_map_.lower_bound(vertex_type{pos_, coord, 1});
        while (iter_bot->second.poly_id != poly_id) {
            ++iter_bot;
        }
        return iter_bot;
    }

    intv_map_iterator _add_new_intv_point(std::size_t pid, coordinate_type coord, int8_t edge_type,
                                          bool is_head) {
        auto ans = intv_map_.emplace_hint(iter_, vertex_type{pos_, coord, edge_type},
                                          poly_val{is_head, pid});
        _append_point(pid, coord, is_head);
        return ans;
    }

    void _new_poly(coordinate_type coord, int8_t edgel, int8_t edger, bool is_head_l) {
        auto pid = poly_list_.size();
        poly_list_.emplace_back();
        _append_point(pid, coord, false);
        auto next = intv_map_.emplace_hint(iter_, vertex_type{pos_, coord, edger},
                                           poly_val{!is_head_l, pid});
        intv_map_.emplace_hint(next, vertex_type{pos_, coord, edgel}, poly_val{is_head_l, pid});
    }

    void _append_point(std::size_t pid, coordinate_type coord, bool is_front) {
        if (is_front) {
            if constexpr (Orient == orientation_2d::X) {
                poly_list_[pid].points.emplace_front(coord, pos_);
            } else {
                poly_list_[pid].points.emplace_front(pos_, coord);
            }
        } else {
            if constexpr (Orient == orientation_2d::X) {
                poly_list_[pid].points.emplace_back(coord, pos_);
            } else {
                poly_list_[pid].points.emplace_back(pos_, coord);
            }
        }
    }

    coordinate_type _concat_poly(std::size_t head, std::size_t tail, std::size_t offset = 0) {
        poly_list_[head].points.insert(poly_list_[head].points.end(),
                                       poly_list_[tail].points.begin() + offset,
                                       poly_list_[tail].points.end());
        poly_list_[tail].points.clear();
        return poly_list_[head].get_bound(true);
    }

    intv_map_iterator _join_poly(intv_map_iterator iterl, intv_map_iterator iterr) {
        auto pidl = iterl->second.poly_id;
        auto pidr = iterr->second.poly_id;
        auto coord_r = iterr->first.coord(pos_);
        // NOTE: use iterr to check head/tail, because iterl could be 1-elment polygon.
        if (pidl == pidr) {
            // we complete a polygon
            auto is_negative = iterr->second.is_head;
            _append_point(pidl, coord_r, is_negative);

            if (is_negative && fracture_) {
                // TODO: need to refactor to advoid generating self-intersecting polygons,
                // TODO: see polygon_90 algorithm for how that's done.

                // need to fracture a negative polygon
                auto coord_l = iterl->first.coord(pos_);
                if (coord_l == coord_r) {
                    // if this negative polygon closes at a single point at the top
                    // (for example, a triangle hole with point at the top), then
                    // we need to append the current point to both the front and the
                    // back so that the hole fractures properly.
                    _append_point(pidl, coord_r, false);
                }

                // fracture out hole by join with the polygon to the left
                auto frac_iter = std::prev(iterl, 1);
                auto pidf = frac_iter->second.poly_id;
                auto &frac_pts = poly_list_[pidf].points;
                auto coord_frac = frac_iter->first.coord(pos_);
                auto constexpr perp = perpendicular(Orient);
                auto concat_offset = 0;
                // add start point of pidf to pidl
                _append_point(pidl, coord_frac, true);
                if (frac_pts[0][perp] == pos_) {
                    // if pidf has a horizontal edge at pos_, we need to make
                    // sure we don't create double edge
                    concat_offset = (frac_pts.size() > 1 && frac_pts[1][perp] == pos_);
                } else {
                    // the polygon to the left has no points at pos_,
                    // so we need to create intermediate vertex to join the two
                    _append_point(pidl, coord_frac, false);
                }

                // concatenate pidl and pidf, then update intervals
                frac_iter->second.poly_id = pidl;
                auto coord_o = _concat_poly(pidl, pidf, concat_offset);
                auto itero = _find_other_poly(coord_o, pidf);
                itero->second.poly_id = pidl;
            } else {
                // positive polygon, or fracture is false and we can output negative polygons
                auto &pt_vec = poly_list_[pidl].points;
                *output_ =
                    get_traits_t<ValueType>::construct(pt_vec.begin(), pt_vec.end(), pt_vec.size());
                ++output_;
                pt_vec.clear();
            }
            intv_map_.erase(iterl);
            intv_map_.erase(iterr++);
        } else {
            // we need to concatenate two polygons together
            auto num_pts_l = poly_list_[pidl].points.size();
            if (iterr->second.is_head) {
                _append_point(pidl, coord_r, false);
                auto coord_o = _concat_poly(pidl, pidr);
                auto itero = _find_other_poly(coord_o, pidr);
                itero->second.poly_id = pidl;
                if (num_pts_l > 1) {
                    // erase left iter only if it points to the tail (and not the head)
                    intv_map_.erase(iterl);
                }
            } else {
                _append_point(pidr, coord_r, false);
                auto coord_o = _concat_poly(pidr, pidl);
                if (num_pts_l > 1) {
                    // other coordinate is only valid if the left polygon has more than 1 point.
                    auto itero = _find_other_poly(coord_o, pidl);
                    itero->second.poly_id = pidr;
                    intv_map_.erase(iterl);
                } else {
                    // if left polygon has 1 point, we need to rename poly ID
                    iterl->second.poly_id = pidr;
                }
            }
            intv_map_.erase(iterr++);
        }
        while (!poly_list_.empty() && poly_list_.back().points.empty()) {
            poly_list_.pop_back();
        }
        return iterr;
    }
};

template <typename T, typename CntType, orientation_2d Orient> class boolean_op {
  public:
    using coordinate_type = T;
    using vertex_type = poly_vertex<coordinate_type>;
    using ivertex_type = intersect_vertex<coordinate_type>;
    using intv_map_type = std::map<vertex_type, CntType>;
    using intv_map_iterator = typename intv_map_type::iterator;
    using output_type = std::vector<polygon_45_set_vertex<coordinate_type, Orient>>;
    using queue_type =
        std::priority_queue<ivertex_type, std::vector<ivertex_type>, std::greater<ivertex_type>>;

  private:
    output_type &output_;
    queue_type intersect_queue_;
    intv_map_type intv_map_;
    intv_map_iterator iter_;
    coordinate_type pos_ = 0;
    CntType delta_{0};

  public:
    explicit boolean_op(output_type &output) : output_(output), iter_(intv_map_.end()) {}

    void process_vertex(coordinate_type pos, coordinate_type coord,
                        const std::array<edge_cnt<CntType>, 2> &edges) {
        if (delta_ != 0) {
            // the previous vertex started a horizontal edge
            // we must process intersections with all down edges before this point
            auto stop = intv_map_.end();
            auto zero_cnt = std::array<edge_cnt<CntType>, 2>{};
            while (iter_ != stop && iter_->first.coord(pos_) < coord) {
                _process_node(pos, iter_->first.coord(pos_), zero_cnt);
            }
        } else {
            // handle all intersection points in queue
            while (_process_intersection(pos, coord)) {
            }
        }
        _process_node(pos, coord, edges);
    }

  private:
    void _update_pos(coordinate_type pos) {
        if (pos != pos_) {
            iter_ = intv_map_.begin();
            pos_ = pos;
        }
    }

    void _update_coord(coordinate_type coord) {
        auto stop = intv_map_.end();
        if (iter_ != stop && iter_->first.coord(pos_) < coord) {
            iter_ = intv_map_.lower_bound(vertex_type(pos_, coord, 1));
        }
    }

    void _append_vertex(coordinate_type pos, coordinate_type coord, bool is_up, int8_t edge_type,
                        bool in_edge) {
        if (!output_.empty() && output_.back().pos() == pos && output_.back().coord() == coord) {
            output_.back().add_edge(is_up, edge_type, in_edge);
        } else {
            output_.emplace_back(pos, coord, is_up, edge_type, in_edge);
        }
    }

    std::size_t _count_down_edges(coordinate_type coord) {
        auto stop = intv_map_.end();
        auto ans = static_cast<std::size_t>(0);
        for (auto cursor = iter_; cursor != stop && cursor->first.coord(pos_) == coord; ++cursor) {
            ++ans;
        }
        return ans;
    }

    bool _process_intersection(coordinate_type vpos, coordinate_type vcoord) {
        if (intersect_queue_.empty())
            return false;

        auto p = intersect_queue_.top();
        auto qpos = p.pos();
        auto qcoord = p.coord();
        auto invalid = p.invalid();
        if (qpos > vpos) {
            return false;
        }
        if (qpos == vpos) {
            if (invalid || qcoord > vcoord) {
                // for invalid node, real intersection position is 0.5 higher
                // so return false to process all nodes at vpos first
                // if qcoord > vcoord, obviously want to process vertex first
                return false;
            }
            if (qcoord == vcoord) {
                // vertex overlap an intersection point, so we remove the
                // intersection and proceed to process the vertex
                intersect_queue_.pop();
                return false;
            }
        }

        // process the intersection point, pop it.
        // don't update coordinate yet, as for invalid intersection we don't need to
        _update_pos(qpos);
        intersect_queue_.pop();

        if (invalid) {
            _process_off_grid_intersection(qcoord);
        } else {
            _update_coord(qcoord);

            auto num_dn = _count_down_edges(qcoord);
            if (num_dn >= 2) {
                // need to process this intersection point before the vertex
                _process_node(qpos, qcoord, std::array<edge_cnt<CntType>, 2>{});
            }
            // this is not an intersection as it has less than 2 edges, do nothing
        }
        return true;
    }

    void _process_node(coordinate_type pos, coordinate_type coord,
                       const std::array<edge_cnt<CntType>, 2> &edges) {
        _update_pos(pos);
        _update_coord(coord);

        // process down edges first
        auto up_edges = edges[1];
        auto stop = intv_map_.end();
        auto pre_cnt1 = (iter_ == intv_map_.begin()) ? CntType{0} : std::prev(iter_, 1)->second;
        auto pre_cnt0 = pre_cnt1 - delta_;
        auto pre_sgn0 = pre_cnt0 > 0;
        auto pre_sgn1 = pre_cnt1 > 0;
        auto cur_cnt0 = pre_cnt0;
        auto cur_cnt1 = pre_cnt1;
        auto cur_sgn0 = pre_sgn0;
        auto cur_sgn1 = pre_sgn1;

        // process and remove down edges
        // all down edges are remove first, so we don't have to worry about order issues when
        // inserting up edges
        std::size_t num_dn = 0;
        std::array<int8_t, 3> dn_edge_arr;
        std::array<bool, 3> dn_in_arr;
        for (; iter_ != stop && iter_->first.coord(pos_) == coord; intv_map_.erase(iter_++)) {
            auto edge_type = iter_->first.edge_type();
            auto new_cnt0 = iter_->second;
            auto new_sgn0 = new_cnt0 > 0;

            if (new_sgn0 ^ cur_sgn0) {
                // register this down edge
                dn_edge_arr[num_dn] = edge_type;
                dn_in_arr[num_dn] = !new_sgn0;
                ++num_dn;
            }
            // subtract old down edge from up edges
            up_edges.add_edge(edge_type, new_cnt0 - cur_cnt0);
            cur_cnt0 = new_cnt0;
            cur_sgn0 = new_sgn0;
        }

        // process up edges
        auto edge0_iter = stop;
        auto edge1_iter = edge0_iter;
        std::size_t num_up = 0;
        std::array<int8_t, 3> up_edge_arr;
        std::array<bool, 3> up_in_arr;
        for (std::size_t idx = 0; idx < 3; ++idx) {
            auto edge_type = static_cast<int8_t>(idx - 1);
            // add down edge to up edge.
            // if this down edge is recorded before, adding it here just cancels the old
            // one out.  If this down edge is not recorded before, that means we have
            // a node with self-cancelled edges before, and the current down edge will
            // annihilate the old down edge and reveal a new up edge.
            up_edges.add_edge(edge_type, edges[0].edge_type_count(edge_type));
            auto cur_edge_cnt = up_edges.count(idx);
            if (cur_edge_cnt != 0) {
                auto new_cnt1 = cur_cnt1 + cur_edge_cnt;
                auto new_sgn1 = new_cnt1 > 0;
                // insert up edge
                edge1_iter =
                    intv_map_.emplace_hint(iter_, poly_vertex{pos, coord, edge_type}, new_cnt1);
                if (edge0_iter == stop)
                    edge0_iter = edge1_iter;
                if (cur_sgn1 ^ new_sgn1) {
                    up_edge_arr[num_up] = edge_type;
                    up_in_arr[num_up] = new_sgn1;
                    ++num_up;
                }
                cur_cnt1 = new_cnt1;
                cur_sgn1 = new_sgn1;
            }
        }

        // append vertex edges only if this is not a vertical/diagonal collinear vertex, and
        // we have at least one up edge or down edge
        if (!(num_up == 1 && num_dn == 1 && up_edge_arr[0] == dn_edge_arr[0] &&
              pre_sgn0 == pre_sgn1) &&
            (num_dn > 0 || num_up > 0)) {
            // this is a valid node, record edges
            for (std::size_t idx = 0; idx < num_dn; ++idx) {
                _append_vertex(pos, coord, false, dn_edge_arr[idx], dn_in_arr[idx]);
            }
            for (std::size_t idx = 0; idx < num_up; ++idx) {
                _append_vertex(pos, coord, true, up_edge_arr[idx], up_in_arr[idx]);
            }
            if (pre_sgn0 ^ pre_sgn1) {
                output_.back().add_h_edge(false, pre_sgn1);
            }
            if (cur_sgn0 ^ cur_sgn1) {
                output_.back().add_h_edge(true, !cur_sgn1);
            }
        }
        delta_ = cur_cnt1 - cur_cnt0;

        if (edge0_iter != stop) {
            // at least one up edge added, compute intersections
            if (edge0_iter != intv_map_.begin()) {
                _compute_intersect(std::prev(edge0_iter, 1), edge0_iter);
            }
            if (edge1_iter != stop) {
                _compute_intersect(edge1_iter, std::next(edge1_iter, 1));
            }
        }
    }

    void _process_off_grid_intersection(coordinate_type coord) {
        auto edge_l = intv_map_.lower_bound(vertex_type(pos_, coord, 1));
        auto stop = intv_map_.end();
        if (edge_l == stop || edge_l->first.coord(pos_) != coord || edge_l->first.edge_type() != 1)
            return;
        auto edge_r = edge_l;
        while (edge_r != stop && edge_r->first.coord(pos_) == coord) {
            ++edge_r;
        }
        if (edge_r != stop && edge_r->first.coord(pos_) == coord + 1 &&
            edge_r->first.edge_type() == -1) {
            auto cnt_l = edge_l->second;
            auto cnt_r = edge_r->second;
            // we really have an off-grid intersection
            if ((cnt_l > 0) ^ (cnt_r > 0)) {
                // this is an off-grid vertex that needs tobe recorded, raise error
                throw std::runtime_error("Off-grid 45-degree vertex detected.");
            } else {
                // this off-grid intersection is not visible, so it's okay
                // but we need to swap vertex order

                // first update poly interval count
                auto pre_cnt =
                    (edge_l == intv_map_.begin()) ? CntType{0} : std::prev(edge_l, 1)->second;
                edge_l->second = pre_cnt + cnt_r - cnt_l;

                // remove both nodes, then add them both back, so order won't be messed up
                auto marker = std::next(edge_r, 1);
                auto node_l = intv_map_.extract(edge_l);
                auto node_r = intv_map_.extract(edge_r);
                node_l.key().set(pos_ + 1, coord, -1);
                node_r.key().set(pos_ + 1, coord + 1, 1);
                intv_map_.insert(marker, std::move(node_l));
                intv_map_.insert(marker, std::move(node_r));
            }
        }
    }

    void _compute_intersect(intv_map_iterator iter0, intv_map_iterator iter1) {
        auto intersect = iter0->first.get_intersection(pos_, iter1->first);
        if (intersect) {
            intersect_queue_.emplace(*intersect);
        }
    }
};

} // namespace detail_poly45
} // namespace polygon
} // namespace cbag

#endif

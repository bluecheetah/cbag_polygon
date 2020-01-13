// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_SET_DATA_H
#define CBAG_POLYGON_POLYGON_SET_DATA_H

#include <algorithm>
#include <vector>

#include <cbag/polygon/polygon_45_data.h>
#include <cbag/polygon/polygon_45_set_detail.h>
#include <cbag/polygon/polygon_90_data.h>
#include <cbag/polygon/polygon_90_set_detail.h>
#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/rectangle_data.h>
#include <cbag/polygon/tag.h>
#include <cbag/util/iterators.h>

namespace cbag {
namespace polygon {

template <typename T, orientation_2d Orient = orientation_2d::HORIZONTAL> class polygon_set_data {
  public:
    using coordinate_type = T;
    using vertex_type = detail_poly45::polygon_45_set_vertex<coordinate_type, Orient>;
    using value_type = std::vector<vertex_type>;
    using const_iterator = typename value_type::const_iterator;

  private:
    polygon_set_type type_ = polygon_set_type::POLY90;
    mutable value_type data_;
    mutable bool dirty_ = false;
    mutable bool unsorted_ = false;

  public:
    polygon_set_data() = default;

    polygon_set_data(polygon_set_type type, value_type data)
        : type_(type), data_(std::move(data)), dirty_(true), unsorted_(true) {}

    template <typename R, IsRectangle<R> = 0>
    explicit polygon_set_data(R &&rect, orientation_2d arr_orient = orientation_2d::X,
                              std::size_t nt = 1, std::size_t np = 1, coordinate_type spt = 0,
                              coordinate_type spp = 0) {
        insert(std::forward<R>(rect), arr_orient, nt, np, spt, spp);
    }

    bool operator==(const polygon_set_data &p) const {
        clean();
        p.clean();
        return data_ == p.data_;
    }

    polygon_set_type polygon_type() const noexcept { return type_; }

    const_iterator begin() const noexcept { return data_.begin(); }

    const_iterator end() const noexcept { return data_.end(); }

    bool empty() const {
        clean();
        return data_.empty();
    }

    std::size_t size() const {
        clean();
        return data_.size();
    }

    void clean() const {
        _sort();
        if (dirty_) {
            std::vector<vertex_type> output;
            output.reserve(data_.size());

            switch (type_) {
            case polygon_set_type::POLY90: {
                auto op = detail_poly90::boolean_op<coordinate_type, int, Orient>(output);

                auto prev_pos = data_[0].pos();
                auto prev_coord = data_[0].coord();
                auto edge_cnt = std::array<int, 2>{0, 0};
                for (const auto &v : data_) {
                    auto pos = v.pos();
                    auto coord = v.coord();
                    if (pos == prev_pos && coord == prev_coord) {
                        edge_cnt[0] += v.vertical_edge_count(false);
                        edge_cnt[1] += v.vertical_edge_count(true);
                    } else {
                        op.process_vertex(prev_pos, prev_coord, edge_cnt);
                        edge_cnt[0] = v.vertical_edge_count(false);
                        edge_cnt[1] = v.vertical_edge_count(true);
                        prev_pos = pos;
                        prev_coord = coord;
                    }
                }
                op.process_vertex(prev_pos, prev_coord, edge_cnt);
                break;
            }
            case polygon_set_type::POLY45: {
                auto op = detail_poly45::boolean_op<coordinate_type, int, Orient>(output);

                auto prev_pos = data_[0].pos();
                auto prev_coord = data_[0].coord();
                auto edge_cnt = std::array<detail_poly45::edge_cnt<int>, 2>{};

                for (const auto &v : data_) {
                    auto pos = v.pos();
                    auto coord = v.coord();
                    if (pos == prev_pos && coord == prev_coord) {
                        edge_cnt[0] += v.edge_info(false);
                        edge_cnt[1] += v.edge_info(true);

                    } else {
                        op.process_vertex(prev_pos, prev_coord, edge_cnt);
                        edge_cnt[0] = v.edge_info(false);
                        edge_cnt[1] = v.edge_info(true);
                        prev_pos = pos;
                        prev_coord = coord;
                    }
                }
                op.process_vertex(prev_pos, prev_coord, edge_cnt);
                break;
            }
            case polygon_set_type::POLY:
                // TODO: implement poly clean
            default:
                throw std::runtime_error("Unsupported poly set type: " + to_string(type_));
            }

            data_ = std::move(output);
            dirty_ = false;
        }
    }

    void clear() {
        data_.clear();
        dirty_ = unsorted_ = false;
    }

    void reserve(std::size_t capacity) { data_.reserve(capacity); }

    void inc_capacity(std::size_t increment) { data_.reserve(data_.size() + increment); }

    void insert(const polygon_set_data &rhs) {
        if (data_.empty())
            *this = rhs;
        else if (!rhs.data_.empty()) {
            dirty_ = unsorted_ = true;
            type_ = max(type_, rhs.type_);
            data_.reserve(data_.size() + rhs.data_.size());
            data_.insert(data_.end(), rhs.data_.begin(), rhs.data_.end());
        }
    }

    void insert(polygon_set_data &&rhs) {
        if (data_.empty())
            *this = std::move(rhs);
        else if (!rhs.data_.empty()) {
            dirty_ = unsorted_ = true;
            type_ = max(type_, rhs.type_);
            data_.reserve(data_.size() + rhs.data_.size());
            data_.insert(data_.end(), rhs.data_.begin(), rhs.data_.end());
        }
    }

    void insert_transform(const polygon_set_data &rhs,
                          const transformation<coordinate_type> &xform) {
        if (!rhs.data_.empty()) {
            dirty_ = unsorted_ = true;
            type_ = max(type_, rhs.type_);
            data_.reserve(data_.size() + rhs.data_.size());
            for (const auto &v : rhs.data_) {
                data_.emplace_back(v);
                data_.back().transform(xform);
            }
        }
    }

    template <typename GeoType> void insert(const GeoType &obj, bool invert = false) {
        _insert(obj, invert, typename tag<GeoType>::type{});
    }

    template <typename R, IsRectangle<R> = 0>
    void insert(R obj, orientation_2d arr_orient, std::size_t nt, std::size_t np,
                coordinate_type spt, coordinate_type spp) {
        if (nt > 0 && np > 0) {
            if (nt == 1 && np == 1) {
                _insert(obj, false, rectangle_tag{});
            } else {
                auto num_arr = std::array<std::size_t, 2>{0, 0};
                auto sp_arr = std::array<coordinate_type, 2>{0, 0};
                auto ocode = to_int(arr_orient);
                num_arr[ocode] = nt;
                num_arr[ocode ^ 1] = np;
                sp_arr[ocode] = spt;
                sp_arr[ocode ^ 1] = spp;

                // fix space sign
                if (sp_arr[0] < 0) {
                    cbag::polygon::move_by(obj, sp_arr[0] * (num_arr[0] - 1), 0);
                    sp_arr[0] = -sp_arr[0];
                }
                if (sp_arr[1] < 0) {
                    cbag::polygon::move_by(obj, 0, sp_arr[1] * (num_arr[1] - 1));
                    sp_arr[1] = -sp_arr[1];
                }

                ocode = to_int(Orient);
                auto dirty_now = !data_.empty();
                _append_rect_array(obj, num_arr[ocode], num_arr[ocode ^ 1], sp_arr[ocode],
                                   sp_arr[ocode ^ 1]);
                if (dirty_now) {
                    dirty_ = unsorted_ = true;
                }
            }
        }
    }

    template <typename GeoType> polygon_set_data &invert(const GeoType &obj) {
        for (auto &vertex : data_) {
            vertex.invert();
        }
        _insert(obj, false, typename tag<GeoType>::type{});
        return *this;
    }

    template <typename OutIter, typename ValueType = polygon_45_data<coordinate_type>>
    void get_polygons(OutIter out_iter, bool fracture = true) const {
        clean();
        if (!data_.empty()) {
            switch (type_) {
            case polygon_set_type::POLY90: {
                auto op = detail_poly90::get_poly<coordinate_type, OutIter, ValueType, Orient>(
                    out_iter, fracture);
                for (const auto &v : data_) {
                    op.process_vertex(v);
                }
                break;
            }
            case polygon_set_type::POLY45: {
                auto op = detail_poly45::get_poly<coordinate_type, OutIter, ValueType, Orient>(
                    out_iter, fracture);
                for (const auto &v : data_) {
                    op.process_vertex(v);
                }
                break;
            }
            case polygon_set_type::POLY:
                // TODO: implement poly get_polygons
            default:
                throw std::runtime_error("Unsupported poly set type: " + to_string(type_));
            }
        }
    }

    polygon_set_data &move_by(coordinate_type dx, coordinate_type dy) {
        if (dx != 0 || dy != 0) {
            for (auto &vertex : data_) {
                vertex.move_by(dx, dy);
            }
        }
        return *this;
    }

    polygon_set_data &transform(const transformation<coordinate_type> &xform) {
        auto orient = xform.orient();
        if (orient == orientation::R0) {
            // this is just translation
            move_by(x(xform), y(xform));
        } else {
            unsorted_ = true;
            for (auto &vertex : data_) {
                vertex.transform(xform);
            }
        }
        return *this;
    }

    polygon_set_data &operator&=(const polygon_set_data &rhs) {
        if (data_.empty() || rhs.data_.empty()) {
            data_.clear();
            dirty_ = false;
            unsorted_ = false;
            return *this;
        }

        _set_boolean_op<detail_poly45::and_cnt>(rhs);
        return *this;
    }

    polygon_set_data &operator-=(const polygon_set_data &rhs) {
        if (data_.empty() || rhs.data_.empty()) {
            return *this;
        }

        _set_boolean_op<detail_poly45::minus_cnt>(rhs);
        return *this;
    }

    template <typename R, IsRectangle<R> = 0> polygon_set_data &convolve_rect(const R &rect) {
        auto rect_coords = std::array<coordinate_type, 4>{xl(rect), yl(rect), xh(rect), yh(rect)};
        if (is_valid(rect)) {
            // convolving with non-negative rectangle
            _convolve_rect_pos(rect_coords);

        } else {
            // NOTE: right now we only support totally negative rectangle
            std::swap(rect_coords[0], rect_coords[2]);
            std::swap(rect_coords[1], rect_coords[3]);
            if (rect_coords[0] > rect_coords[2] || rect_coords[1] > rect_coords[3]) {
                throw std::runtime_error("Currently cannot convolve with semi-negative rectangle.");
            }
            auto tot_box = get_bbox(*this);
            auto max_x = std::max(std::abs(rect_coords[0]), std::abs(rect_coords[2]));
            auto max_y = std::max(std::abs(rect_coords[1]), std::abs(rect_coords[3]));
            expand(tot_box, max_x, max_y);
            invert(tot_box);
            _convolve_rect_pos(rect_coords);
            invert(tot_box);
        }
        return *this;
    }

  private:
    void _sort() const {
        if (unsorted_) {
            std::sort(data_.begin(), data_.end());
            unsorted_ = false;
        }
    }

    void _convolve_rect_pos(const std::array<coordinate_type, 4> &rect_coords) {
        clean();
        if (!data_.empty()) {
            decltype(data_) new_data;

            switch (type_) {
            case polygon_set_type::POLY90: {
                using V = polygon_90_data<coordinate_type>;

                auto out_iter =
                    cbag::util::lambda_output_iterator([this, &rect_coords, &new_data](V &&v) {
                        auto start = v.begin_compact();
                        auto stop = v.end_compact();

                        auto itrb =
                            poly90_convolve_rect_coord_iterator<decltype(start), coordinate_type>(
                                start, stop, rect_coords[0], rect_coords[1], rect_coords[2],
                                rect_coords[3]);
                        auto itre = decltype(itrb)(stop);

                        _compact_to_vertex(new_data, v.size(), itrb, itre, false);
                    });
                get_polygons<decltype(out_iter), V>(std::move(out_iter), true);
                break;
            }
            case polygon_set_type::POLY45: {
                using V = polygon_45_data<coordinate_type>;

                auto out_iter =
                    cbag::util::lambda_output_iterator([this, &rect_coords, &new_data](V &&v) {
                        auto start = v.begin();
                        auto stop = v.end();

                        auto itrb = poly45_convolve_rect_iterator<decltype(start), coordinate_type>(
                            start, stop, rect_coords[0], rect_coords[1], rect_coords[2],
                            rect_coords[3]);
                        auto itre = decltype(itrb)(stop);

                        _point_to_vertex(new_data, v.size(), itrb, itre, false);
                    });
                // TODO: correct this after it is fixed
                throw std::runtime_error(
                    "convolve_rect for poly45 is broken now.  Contact developer.");
                get_polygons<decltype(out_iter), V>(std::move(out_iter), true);
                break;
            }
            case polygon_set_type::POLY: {
                // TODO: implement convolve_rect for poly
            }
            default:
                throw std::runtime_error("Unsupported poly set type: " + to_string(type_));
            }

            data_ = std::move(new_data);
            dirty_ = unsorted_ = !data_.empty();
        }
    }

    template <typename poly_type> void _insert(const poly_type &obj, bool invert, rectangle_tag) {
        if (is_physical(obj)) {
            auto perp = perpendicular(Orient);
            auto coord0 = lower(obj, Orient);
            auto coord1 = upper(obj, Orient);
            auto pos0 = lower(obj, perp);
            auto pos1 = upper(obj, perp);
            auto constexpr x_first = Orient == orientation_2d::X;
            auto is_empty = data_.empty();
            data_.reserve(data_.size() + 4);
            data_.emplace_back(pos1, coord0, pos0, coord1, !x_first, invert);
            data_.emplace_back(coord0, pos0, coord1, pos1, x_first, invert);
            data_.emplace_back(coord1, pos1, coord0, pos0, x_first, invert);
            data_.emplace_back(pos0, coord1, pos1, coord0, !x_first, invert);
            if (!is_empty) {
                dirty_ = unsorted_ = true;
            }
        }
    }

    template <typename R, IsRectangle<R> = 0>
    void _append_rect_array(const R &rect, std::size_t nt, std::size_t np, coordinate_type spt,
                            coordinate_type spp) {
        if (is_physical(rect)) {
            auto perp = perpendicular(Orient);
            auto coord0 = lower(rect, Orient);
            auto coord1 = upper(rect, Orient);
            auto pos0 = lower(rect, perp);
            auto pos1 = upper(rect, perp);

            // fix overlapping rectangles
            if (nt > 1 && spt <= coord1 - coord0) {
                coord1 += (nt - 1) * spt;
                nt = 1;
            }
            if (np > 1 && spp <= pos1 - pos0) {
                pos1 += (np - 1) * spp;
                np = 1;
            }

            auto constexpr x_first = Orient == orientation_2d::X;
            data_.reserve(data_.size() + nt * np * 4);
            auto dp = static_cast<coordinate_type>(0);
            for (std::size_t idxp = 0; idxp < np; ++idxp, dp += spp) {
                auto p0_cur = pos0 + dp;
                auto p1_cur = pos1 + dp;
                auto dt = static_cast<coordinate_type>(0);
                for (std::size_t idxt = 0; idxt < nt; ++idxt, dt += spt) {
                    data_.emplace_back(p1_cur, coord0 + dt, p0_cur, coord1 + dt, !x_first, false);
                    data_.emplace_back(coord0 + dt, p0_cur, coord1 + dt, p1_cur, x_first, false);
                }
                dt = 0;
                for (std::size_t idxt = 0; idxt < nt; ++idxt, dt += spt) {
                    data_.emplace_back(coord1 + dt, p1_cur, coord0 + dt, p0_cur, x_first, false);
                    data_.emplace_back(p0_cur, coord1 + dt, p1_cur, coord0 + dt, !x_first, false);
                }
            }
        }
    }

    template <typename poly_type> void _insert(const poly_type &obj, bool invert, polygon_90_tag) {
        auto n = cbag::polygon::size(obj);
        auto tmp = _compact_to_vertex(data_, n, begin_compact(obj), end_compact(obj), invert);
        dirty_ |= tmp;
        unsorted_ |= tmp;
    }

    template <typename poly_type> void _insert(const poly_type &obj, bool invert, polygon_tag) {
        auto n = cbag::polygon::size(obj);
        auto[tmp, is_45] = _point_to_vertex(data_, n, begin_points(obj), end_points(obj), invert);
        dirty_ |= tmp;
        unsorted_ |= tmp;
        if (is_45)
            type_ = max(type_, polygon_set_type::POLY45);
    }

    template <typename Iter>
    bool _compact_to_vertex(value_type &vec, std::size_t n, Iter start, Iter stop, bool invert) {
        if (n < 4 || start == stop)
            return false;

        std::array<coordinate_type, 7> coords;
        coords[0] = coords[4] = *start;
        ++start;
        coords[1] = coords[5] = *start;
        ++start;
        coords[2] = coords[6] = *start;
        ++start;
        coords[3] = *start;
        ++start;

        auto x_first = true;
        vec.reserve(vec.size() + n);
        // add fake 1/-1 increment so edges of duplicate points subtract out
        vec.emplace_back(coords[0] - (coords[0] == coords[2]), coords[1], coords[2],
                         coords[3] + (coords[3] == coords[1]), x_first, invert);
        for (; start != stop; ++start) {
            coords[0] = coords[1];
            coords[1] = coords[2];
            coords[2] = coords[3];
            coords[3] = *start;
            x_first ^= 1;
            vec.emplace_back(coords[0] + (coords[0] == coords[2]), coords[1], coords[2],
                             coords[3] + (coords[3] == coords[1]), x_first, invert);
        }
        x_first ^= 1;
        vec.emplace_back(coords[1] + (coords[1] == coords[3]), coords[2], coords[3],
                         coords[4] + (coords[4] == coords[2]), x_first, invert);
        x_first ^= 1;
        vec.emplace_back(coords[2] + (coords[2] == coords[4]), coords[3], coords[4],
                         coords[5] + (coords[5] == coords[3]), x_first, invert);
        x_first ^= 1;
        vec.emplace_back(coords[3] + (coords[3] == coords[5]), coords[4], coords[5],
                         coords[6] - (coords[6] == coords[4]), x_first, invert);
        return true;
    }

    template <typename Iter>
    std::array<bool, 2> _point_to_vertex(value_type &vec, std::size_t n, Iter start, Iter stop,
                                         bool invert) {
        auto istart = dispatch::poly_reduce_iterator(start, stop);

        if (!istart.has_next())
            return std::array<bool, 2>{false, false};

        auto is_45 = false;
        vec.reserve(vec.size() + n);
        std::array<typename dispatch::poly_reduce_iterator<Iter>::value_type, 5> pts;
        pts[0] = pts[3] = *istart;
        ++istart;
        if (!istart.has_next())
            return std::array<bool, 2>{false, false};
        pts[1] = pts[4] = *istart;
        ++istart;
        if (!istart.has_next())
            return std::array<bool, 2>{false, false};
        pts[2] = *istart;
        ++istart;
        vec.emplace_back(x(pts[0]), y(pts[0]), x(pts[1]), y(pts[1]), x(pts[2]), y(pts[2]), invert);
        is_45 |= vec.back().has_diagonal();
        for (; istart.has_next(); ++istart) {
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts[2] = *istart;
            vec.emplace_back(x(pts[0]), y(pts[0]), x(pts[1]), y(pts[1]), x(pts[2]), y(pts[2]),
                             invert);
            is_45 |= vec.back().has_diagonal();
        }
        vec.emplace_back(x(pts[1]), y(pts[1]), x(pts[2]), y(pts[2]), x(pts[3]), y(pts[3]), invert);
        is_45 |= vec.back().has_diagonal();
        vec.emplace_back(x(pts[2]), y(pts[2]), x(pts[3]), y(pts[3]), x(pts[4]), y(pts[4]), invert);
        is_45 |= vec.back().has_diagonal();
        return std::array<bool, 2>{true, is_45};
    }

    // precondition: both *this and rhs are non-empty
    template <typename CntType> void _set_boolean_op(const polygon_set_data &rhs) {
        rhs.clean();
        clean();

        auto num_l = data_.size();
        auto num_r = rhs.data_.size();
        auto idx_l = static_cast<std::size_t>(0);
        auto idx_r = static_cast<std::size_t>(0);

        std::vector<vertex_type> output;
        output.reserve(num_l + num_r);

        // get first point
        coordinate_type prev_pos, prev_coord;
        if (data_[0] <= rhs.data_[0]) {
            prev_pos = data_[0].pos();
            prev_coord = data_[0].coord();
        } else {
            prev_pos = rhs.data_[0].pos();
            prev_coord = rhs.data_[0].coord();
        }

        switch (type_) {
        case polygon_set_type::POLY90: {
            auto op = detail_poly90::boolean_op<coordinate_type, CntType, Orient>(output);
            auto edge_cnt = std::array<CntType, 2>{CntType{0}, CntType{0}};
            auto done = false;
            while (!done) {
                // add up all vertex at the same location
                for (; idx_l < num_l && data_[idx_l].pos() == prev_pos &&
                       data_[idx_l].coord() == prev_coord;
                     ++idx_l) {
                    edge_cnt[0].increment(false, data_[idx_l].vertical_edge_count(false));
                    edge_cnt[1].increment(false, data_[idx_l].vertical_edge_count(true));
                }
                for (; idx_r < num_r && rhs.data_[idx_r].pos() == prev_pos &&
                       rhs.data_[idx_r].coord() == prev_coord;
                     ++idx_r) {
                    edge_cnt[0].increment(true, rhs.data_[idx_r].vertical_edge_count(false));
                    edge_cnt[1].increment(true, rhs.data_[idx_r].vertical_edge_count(true));
                }

                // process
                op.process_vertex(prev_pos, prev_coord, edge_cnt);

                // update location
                edge_cnt[0] = 0;
                edge_cnt[1] = 0;
                if (idx_l < num_l) {
                    if (idx_r < num_r && rhs.data_[idx_r] <= data_[idx_l]) {
                        prev_pos = rhs.data_[idx_r].pos();
                        prev_coord = rhs.data_[idx_r].coord();
                    } else {
                        prev_pos = data_[idx_l].pos();
                        prev_coord = data_[idx_l].coord();
                    }
                } else if (idx_r < num_r) {
                    prev_pos = rhs.data_[idx_r].pos();
                    prev_coord = rhs.data_[idx_r].coord();
                } else {
                    done = true;
                }
            }

            break;
        }
        case polygon_set_type::POLY45: {
            auto op = detail_poly45::boolean_op<coordinate_type, CntType, Orient>(output);
            auto edge_cnt = std::array<detail_poly45::edge_cnt<CntType>, 2>{};
            auto done = false;
            while (!done) {
                // add up all vertex at the same location
                for (; idx_l < num_l && data_[idx_l].pos() == prev_pos &&
                       data_[idx_l].coord() == prev_coord;
                     ++idx_l) {
                    edge_cnt[0].add_count(false, data_[idx_l].edge_info(false));
                    edge_cnt[1].add_count(false, data_[idx_l].edge_info(true));
                }
                for (; idx_r < num_r && rhs.data_[idx_r].pos() == prev_pos &&
                       rhs.data_[idx_r].coord() == prev_coord;
                     ++idx_r) {
                    edge_cnt[0].add_count(true, rhs.data_[idx_r].edge_info(false));
                    edge_cnt[1].add_count(true, rhs.data_[idx_r].edge_info(true));
                }

                // process
                op.process_vertex(prev_pos, prev_coord, edge_cnt);

                // update location
                edge_cnt[0] = 0;
                edge_cnt[1] = 0;
                if (idx_l < num_l) {
                    if (idx_r < num_r && rhs.data_[idx_r] <= data_[idx_l]) {
                        prev_pos = rhs.data_[idx_r].pos();
                        prev_coord = rhs.data_[idx_r].coord();
                    } else {
                        prev_pos = data_[idx_l].pos();
                        prev_coord = data_[idx_l].coord();
                    }
                } else if (idx_r < num_r) {
                    prev_pos = rhs.data_[idx_r].pos();
                    prev_coord = rhs.data_[idx_r].coord();
                } else {
                    done = true;
                }
            }

            break;
        }
        case polygon_set_type::POLY:
            // TODO: implement poly boolean op
        default:
            throw std::runtime_error("Unsupported poly set type: " + to_string(type_));
        }

        data_ = std::move(output);
        dirty_ = false;
    }
};

template <typename T> struct tag<polygon_set_data<T>> { using type = polygon_set_tag; };

template <typename T>
bool operator!=(const polygon_set_data<T> &lhs, const polygon_set_data<T> &rhs) {
    return !(lhs == rhs);
}

template <typename T> rectangle_data<T> get_bbox(const polygon_set_data<T> &obj) {
    obj.clean();
    auto ans = rectangle_data<T>::get_invalid_bbox();
    for (const auto &v : obj) {
        extend_to(ans, v.x(), v.y());
    }
    return ans;
}

template <typename T, typename Fun, typename ValueType = polygon_45_data<T>>
void apply_polygons(const polygon_set_data<T> &obj, Fun lambda, bool fracture = true) {
    obj.template get_polygons<cbag::util::lambda_output_iterator<Fun>, ValueType>(
        cbag::util::lambda_output_iterator<Fun>(std::move(lambda)), fracture);
}

template <typename T, typename R, IsRectangle<R> = 0>
polygon_set_data<T> get_convolve_rect(const polygon_set_data<T> &obj, const R &rect) {
    obj.clean();
    auto ans = obj;
    return ans.convolve_rect(rect);
}

template <typename T> polygon_set_data<T> &expand(polygon_set_data<T> &obj, T dx, T dy) {
    return obj.convolve_rect(rectangle_data<T>(-dx, -dy, dx, dy));
}

template <typename T> polygon_set_data<T> get_expand(const polygon_set_data<T> &obj, T dx, T dy) {
    obj.clean();
    auto ans = obj;
    return expand(ans, dx, dy);
}

template <typename T> polygon_set_data<T> get_holes(const polygon_set_data<T> &obj) {
    auto box = get_bbox(obj);
    auto inv_box = get_expand(box, 1, 1);

    auto tmp = obj;
    tmp.invert(inv_box);

    auto ans = polygon_set_data<T>();
    ans.reserve(tmp.size() - 1);
    apply_polygons(tmp, [&ans, &box](const auto &p) {
        if (contains(box, p)) {
            ans.insert(p);
        }
    });

    return ans;
}

template <typename T>
polygon_set_data<T> &operator|=(polygon_set_data<T> &lhs, const polygon_set_data<T> &rhs) {
    lhs.clean();
    rhs.clean();
    lhs.insert(rhs);
    return lhs;
}

template <typename T>
polygon_set_data<T> &operator|=(polygon_set_data<T> &lhs, polygon_set_data<T> &&rhs) {
    lhs.clean();
    rhs.clean();
    lhs.insert(std::move(rhs));
    return lhs;
}

template <typename T>
polygon_set_data<T> &operator+=(polygon_set_data<T> &lhs, const polygon_set_data<T> &rhs) {
    return (lhs |= rhs);
}

template <typename T>
polygon_set_data<T> &operator+=(polygon_set_data<T> &lhs, polygon_set_data<T> &&rhs) {
    return (lhs |= std::move(rhs));
}

template <typename T>
polygon_set_data<T> &operator*=(polygon_set_data<T> &lhs, const polygon_set_data<T> &rhs) {
    return (lhs &= rhs);
}

template <typename P1, typename P2>
polygon_set_data<coord_type<P1>> operator&(const P1 &lhs, const P2 &rhs) {
    auto ans = polygon_set_data<coord_type<P1>>();
    auto rhs_set = polygon_set_data<coord_type<P1>>();
    ans.insert(lhs);
    rhs_set.insert(rhs);
    ans &= rhs_set;
    return ans;
}

template <typename P1, typename P2>
polygon_set_data<coord_type<P1>> operator|(const P1 &lhs, const P2 &rhs) {
    auto ans = polygon_set_data<coord_type<P1>>();
    ans.insert(lhs);
    ans.insert(rhs);
    return ans;
}

template <typename P1, typename P2>
polygon_set_data<coord_type<P1>> operator*(const P1 &lhs, const P2 &rhs) {
    return lhs & rhs;
}

template <typename P1, typename P2>
polygon_set_data<coord_type<P1>> operator+(const P1 &lhs, const P2 &rhs) {
    return lhs + rhs;
}

namespace dispatch {

template <typename T1, typename T2>
bool contains(const T1 &container, const T2 &obj, polygon_tag, polygon_tag) {
    auto tmp1 = polygon_set_data<coord_type<T1>>();
    auto tmp2 = polygon_set_data<coord_type<T1>>();
    tmp1.insert(container);
    tmp2.insert(obj);

    auto tmp3 = tmp2;
    tmp3 &= tmp1;
    return tmp2 == tmp3;
}

template <typename T1, typename T2>
bool contains(const T1 &container, const T2 &obj, polygon_set_tag, polygon_tag) {
    auto tmp2 = polygon_set_data<coord_type<T2>>();
    tmp2.insert(obj);

    auto tmp3 = tmp2;
    tmp3 &= container;
    return tmp2 == tmp3;
}

} // namespace dispatch

template <typename T1, typename T2> bool contains(const T1 &container, const T2 &obj) {
    return dispatch::contains(container, obj, typename tag<T1>::type{}, typename tag<T2>::type{});
}

} // namespace polygon
} // namespace cbag

#endif

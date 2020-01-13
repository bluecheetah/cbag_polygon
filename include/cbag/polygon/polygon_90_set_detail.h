// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_90_SET_DETAIL_H
#define CBAG_POLYGON_POLYGON_90_SET_DETAIL_H

// #include <iostream>

#include <algorithm>
#include <deque>
#include <map>
#include <ostream>
#include <vector>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/polygon_45_set_detail.h>
#include <cbag/polygon/tag.h>
#include <cbag/polygon/transformation.h>

namespace cbag {
namespace polygon {
namespace detail_poly90 {

template <typename T, typename OutIter, typename ValueType, orientation_2d Orient> class get_poly {
  public:
    using coordinate_type = T;
    using vertex_type = detail_poly45::polygon_45_set_vertex<coordinate_type, Orient>;

    struct poly_val {
      public:
        bool is_head = false;
        std::size_t poly_id = 0;
    };

    struct poly_info {
      public:
        std::deque<point_data<coordinate_type>> points;
        std::size_t last_idx;

        poly_info() = default;

        coordinate_type get_bound(bool upper) const {
            auto idx = (points.size() - 1) * upper;
            return points[idx][Orient];
        }
    };

    using intv_map_type = std::map<coordinate_type, poly_val>;
    using intv_map_iterator = typename intv_map_type::iterator;

  private:
    OutIter output_;
    intv_map_type intv_map_;
    intv_map_iterator iter_;
    std::vector<poly_info> poly_list_;
    coordinate_type pos_ = 0;
    std::vector<std::size_t> cur_poly_list_;
    bool open_poly_ = false;
    bool fracture_ = true;

  public:
    get_poly(OutIter output, bool fracture)
        : output_(output), iter_(intv_map_.end()), fracture_(fracture) {}

    void process_vertex(const vertex_type &vertex) {
        auto coord = vertex.coord();

        _update_pos(vertex.pos());
        _update_coord(coord);

        auto[has_up, up_in] = vertex.get_vertical_up_info();
        auto has_dn = (iter_ != intv_map_.end() && iter_->first == coord);

        // process current open polygon
        if (open_poly_) {
            auto open_iter = std::prev(iter_, 1);
            open_poly_ = false;
            if (has_dn) {
                // join polygon with down edge if there's any
                iter_ = _join_poly(open_iter, iter_);
                has_dn = false;
            } else {
                // we are working on clean polygon set, so if there's no down edge,
                // there must be an up edge
                // no down edge means this is a new vertex
                auto pid = open_iter->second.poly_id;
                auto prev_is_head = open_iter->second.is_head;
                if (poly_list_[pid].points.size() == 1) {
                    // adding the second point of poly interval
                    _add_new_intv_point(pid, coord, !prev_is_head);
                } else {
                    // appending to an existing polygon
                    _append_point(pid, coord, prev_is_head);
                    // update the last node
                    auto node = intv_map_.extract(open_iter);
                    node.key() = coord;
                    intv_map_.insert(iter_, std::move(node));
                }
                has_up = false;
                cur_poly_list_.push_back(pid);
            }
        }

        // at this point, there's no left edges, meaning we cannot have both down
        // and up edges.

        if (has_dn) {
            // add current point to poly
            auto pid = iter_->second.poly_id;
            _append_point(pid, coord, iter_->second.is_head);
            // have right edge, set open_poly
            open_poly_ = true;
            cur_poly_list_.push_back(pid);
        }
        if (has_up) {
            // add new open polygon, because we do not have down edge
            auto pid = poly_list_.size();
            poly_list_.emplace_back();
            _add_new_intv_point(pid, coord, up_in);
            // have right edge, set open_poly
            open_poly_ = true;
            cur_poly_list_.push_back(pid);
        }

        // NOTE: debug print statements
        /*
        std::cout << "processed vertex: " << vertex.to_string() << std::endl;
        std::cout << "polygon set details:" << std::endl;
        auto num = poly_list_.size();
        for (std::size_t idx = 0; idx < num; ++idx) {
            auto &v = poly_list_[idx];
            if (!v.points.empty()) {
                std::cout << "poly points for " << idx << std::endl;
                for (const auto &p : v.points) {
                    std::cout << to_string(p) << std::endl;
                }
                std::cout << "poly end" << std::endl;
            }
        }
        */
    }

  private:
    void _update_pos(coordinate_type pos) {
        if (pos_ != pos) {
            pos_ = pos;
            cur_poly_list_.clear();
            iter_ = intv_map_.begin();
        }
    }

    void _update_coord(coordinate_type coord) {
        if (iter_ != intv_map_.end() && iter_->first < coord) {
            iter_ = intv_map_.lower_bound(coord);
        }
    }

    intv_map_iterator _add_new_intv_point(std::size_t pid, coordinate_type coord,
                                          bool append_head) {
        auto ans = intv_map_.emplace_hint(iter_, coord, poly_val{append_head, pid});
        _append_point(pid, coord, append_head);
        return ans;
    }

    void _append_point(std::size_t pid, coordinate_type coord, bool is_front) {
        if (is_front) {
            poly_list_[pid].last_idx = 0;
            if (Orient == orientation_2d::X) {
                poly_list_[pid].points.emplace_front(coord, pos_);
            } else {
                poly_list_[pid].points.emplace_front(pos_, coord);
            }
        } else {
            poly_list_[pid].last_idx = poly_list_[pid].points.size();
            if (Orient == orientation_2d::X) {
                poly_list_[pid].points.emplace_back(coord, pos_);
            } else {
                poly_list_[pid].points.emplace_back(pos_, coord);
            }
        }
    }

    coordinate_type _concat_poly(std::size_t head, std::size_t tail) {
        poly_list_[head].points.insert(poly_list_[head].points.end(),
                                       poly_list_[tail].points.begin(),
                                       poly_list_[tail].points.end());
        poly_list_[tail].points.clear();
        return poly_list_[head].get_bound(true);
    }

    coordinate_type _insert_poly(std::size_t head, std::size_t tail, std::size_t join_idx) {
        poly_list_[head].points.insert(poly_list_[head].points.begin() + join_idx,
                                       poly_list_[tail].points.begin(),
                                       poly_list_[tail].points.end());
        poly_list_[tail].points.clear();
        return poly_list_[head].get_bound(true);
    }

    std::size_t _rotate_neg_poly(std::size_t pid) {
        auto &vec = poly_list_[pid].points;
        auto coord_min = vec[0][Orient];
        auto idx_min = static_cast<std::size_t>(0);
        auto ntot = vec.size();

        auto constexpr perp = perpendicular(Orient);
        for (std::size_t idx = 1; idx < ntot; ++idx) {
            if (vec[idx][perp] == pos_) {
                auto cur_coord = vec[idx][Orient];
                if (cur_coord < coord_min) {
                    coord_min = cur_coord;
                    idx_min = idx;
                }
            }
        }

        if ((idx_min == ntot - 1 || idx_min == 1) && vec[0][Orient] != coord_min) {
            // no rotation required
            return 0;
        } else {
            // get new start index
            auto new_start = ntot - 1;
            if (idx_min != ntot - 1 && vec[idx_min + 1][Orient] != coord_min)
                new_start = idx_min + 1;
            else if (idx_min != 0 && vec[idx_min - 1][Orient] != coord_min)
                new_start = idx_min - 1;

            // we failed previous check, so the new start must be ntot - 2
            auto start = vec.begin();
            std::rotate(start, start + new_start, vec.end());
            return ntot - new_start;
        }
    }

    intv_map_iterator _join_poly(intv_map_iterator iterl, intv_map_iterator iterr) {
        auto pidl = iterl->second.poly_id;
        auto pidr = iterr->second.poly_id;
        auto coord_r = iterr->first;
        // NOTE: use iterr to check head/tail, because iterl could be 1-elment polygon.
        if (pidl == pidr) {
            // we complete a polygon
            auto is_negative = iterr->second.is_head;
            _append_point(pidl, coord_r, is_negative);

            if (is_negative && fracture_) {
                // need to fracture a negative polygon, first, need to find parent polygon
                // candidate 1: a polygon with an up or through edge at pos_
                auto frac_iter = std::prev(iterl, 1);
                auto pidf = frac_iter->second.poly_id;
                // candidate 2: a polygon with a down edge at pos_
                auto pid_test = std::numeric_limits<std::size_t>::max();
                auto pstop = cur_poly_list_.rend();
                for (auto pstart = cur_poly_list_.rbegin(); pstart != pstop; ++pstart) {
                    auto cur_pid = *pstart;
                    if (cur_pid != pidl) {
                        pid_test = cur_pid;
                        break;
                    }
                }
                if (pid_test != std::numeric_limits<std::size_t>::max()) {
                    // there is a polygon with a down edge at pos_, check to see if it's better
                    auto join_idx_test = poly_list_[pid_test].last_idx;
                    auto coord_test = poly_list_[pid_test].points[join_idx_test][Orient];
                    if (coord_test >= frac_iter->first) {
                        // need to fracture to this polygon
                        pidf = pid_test;
                        auto frac_upper = poly_list_[pid_test].get_bound(true);
                        frac_iter = (frac_upper < coord_r)
                                        ? intv_map_.find(frac_upper)
                                        : intv_map_.find(poly_list_[pidf].get_bound(false));
                    } else {
                        // fracture to candidate 1
                        pid_test = std::numeric_limits<std::size_t>::max();
                    }
                }
                // here, pid_test will be MAX value if we need to fracture to a through edge

                auto join_idx = poly_list_[pidf].last_idx;

                // rotate the negative polygon so that index 0 is the correct insertion point
                auto pidl_last_idx = _rotate_neg_poly(pidl);

                // NOTE: debug print statements
                // std::cout << "got negative poly:" << std::endl;
                // for (const auto &pt : poly_list_[pidl].points) {
                //     std::cout << to_string(pt) << std::endl;
                // }
                // std::cout << "end poly" << std::endl;
                // std::cout << "joining with poly:" << std::endl;
                // for (const auto &pt : poly_list_[pidf].points) {
                //     std::cout << to_string(pt) << std::endl;
                // }
                // std::cout << "end poly" << std::endl;
                // std::cout << "join_idx: " << join_idx
                //           << ", join point: " << to_string(poly_list_[pidf].points[join_idx])
                //           << std::endl;

                if (pid_test != std::numeric_limits<std::size_t>::max()) {
                    // stitch to parent polygon at join_idx
                    if (join_idx == 0) {
                        // we are joining at index 0.  This means the joining point is a head,
                        // and we know this head has a horizontal left edge (because otherwise
                        // the vertices aren't clean).
                        // This means need to join at index 1 to keep up edge at the same place
                        // and not introduce extra vertices
                        join_idx = 1;
                    }
                    poly_list_[pidf].last_idx = join_idx + pidl_last_idx;
                    auto coord_o = _insert_poly(pidf, pidl, join_idx);

                    // update last_idx of final poly.  As is_negative = true,
                    // the last processed vertex is at index 0 of pidl
                    // TODO: may not need to set poly_id to pidf, because they already should
                    // TODO: be.  Remove after more verification
                    frac_iter->second.poly_id = pidf;
                    auto itero = intv_map_.find(coord_o);
                    itero->second.poly_id = pidf;

                    auto new_end_iter =
                        std::remove(cur_poly_list_.begin(), cur_poly_list_.end(), pidl);
                    cur_poly_list_.erase(new_end_iter, cur_poly_list_.end());
                    cur_poly_list_.push_back(pidf);
                } else {
                    // the parent polygon has no vertex at pos_, or the cut will hit a through
                    // edge before hitting the vertex at pos_
                    // we need to create intermediate vertex to join the two
                    auto coord_frac0 = frac_iter->first;
                    _append_point(pidl, coord_frac0, true);
                    _append_point(pidf, coord_frac0, true);
                    frac_iter->second.poly_id = pidl;
                    auto coord_o = _concat_poly(pidl, pidf);
                    auto itero = intv_map_.find(coord_o);
                    itero->second.poly_id = pidl;
                    // update last_idx of final poly.  As is_negative = true,
                    // the last processed vertex is at index 1
                    poly_list_[pidl].last_idx = pidl_last_idx + 1;

                    auto new_end_iter =
                        std::remove(cur_poly_list_.begin(), cur_poly_list_.end(), pidf);
                    cur_poly_list_.erase(new_end_iter, cur_poly_list_.end());
                    cur_poly_list_.push_back(pidl);
                }

                // NOTE: debug print statements
                // auto pidp = frac_iter->second.poly_id;
                // std::cout << "final poly:" << std::endl;
                // for (const auto &pt : poly_list_[pidp].points) {
                //     std::cout << to_string(pt) << std::endl;
                // }
                // std::cout << "end poly" << std::endl;
                // std::cout << "join index: " << poly_list_[pidp].last_idx
                //             << ", poly join point :"
                //           << to_string(poly_list_[pidp].points[poly_list_[pidp].last_idx])
                //           << std::endl;
                // // NOTE: debug exception check
                // for (std::size_t pidx = 0; pidx < poly_list_[pidp].points.size(); ++pidx) {
                //     auto oidx = (pidx == 0) ? poly_list_[pidp].points.size() - 1 : pidx - 1;
                //     if (poly_list_[pidp].points[pidx] == poly_list_[pidp].points[oidx])
                //         throw std::runtime_error("double vertex formed at index " +
                //                                  std::to_string(pidx));
                // }
            } else {
                // positive polygon, or fracture is false and we can output negative polygons
                auto &pt_vec = poly_list_[pidl].points;
                *output_ =
                    get_traits_t<ValueType>::construct(pt_vec.begin(), pt_vec.end(), pt_vec.size());
                ++output_;
                pt_vec.clear();
                auto new_end_iter = std::remove(cur_poly_list_.begin(), cur_poly_list_.end(), pidl);
                cur_poly_list_.erase(new_end_iter, cur_poly_list_.end());
            }
            intv_map_.erase(iterl);
            intv_map_.erase(iterr++);
        } else {
            // we need to concatenate two polygons together
            auto num_pts_l = poly_list_[pidl].points.size();
            if (iterr->second.is_head) {
                _append_point(pidl, coord_r, false);
                auto coord_o = _concat_poly(pidl, pidr);
                auto itero = intv_map_.lower_bound(coord_o);
                itero->second.poly_id = pidl;
                if (num_pts_l > 1) {
                    // if right iter points to head, then left iter must point to tail,
                    // in which case we need to erase left iter.  However, if left
                    // iter has only one point, then it points to both head and tail, hence
                    // the if statement guard
                    intv_map_.erase(iterl);
                }
                auto new_end_iter = std::remove(cur_poly_list_.begin(), cur_poly_list_.end(), pidr);
                cur_poly_list_.erase(new_end_iter, cur_poly_list_.end());
                cur_poly_list_.push_back(pidl);
            } else {
                _append_point(pidr, coord_r, false);
                auto coord_o = _concat_poly(pidr, pidl);
                if (num_pts_l > 1) {
                    // other coordinate is only valid if the left polygon has more than 1 point.
                    auto itero = intv_map_.lower_bound(coord_o);
                    itero->second.poly_id = pidr;
                    intv_map_.erase(iterl);
                } else {
                    // if left polygon has 1 point, we need to rename poly ID
                    iterl->second.poly_id = pidr;
                }
                auto new_end_iter = std::remove(cur_poly_list_.begin(), cur_poly_list_.end(), pidl);
                cur_poly_list_.erase(new_end_iter, cur_poly_list_.end());
                cur_poly_list_.push_back(pidr);
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
    using vertex_type = detail_poly45::polygon_45_set_vertex<coordinate_type, Orient>;
    using intv_map_type = std::map<coordinate_type, CntType>;
    using intv_map_iterator = typename intv_map_type::iterator;
    using output_type = std::vector<vertex_type>;

  private:
    output_type &output_;
    intv_map_type intv_map_;
    intv_map_iterator iter_;
    coordinate_type pos_ = 0;
    CntType delta_{0};

  public:
    explicit boolean_op(output_type &output) : output_(output), iter_(intv_map_.end()) {}

    void process_vertex(coordinate_type pos, coordinate_type coord,
                        const std::array<CntType, 2> &edges) {
        if (delta_ != 0) {
            // the previous vertex started a horizontal edge
            // we must process intersections with all down edges before this point
            auto stop = intv_map_.end();
            auto zero_cnt = std::array<CntType, 2>{CntType{0}, CntType{0}};
            while (iter_ != stop && iter_->first < coord) {
                _process_node(pos, iter_->first, zero_cnt);
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
        if (iter_ != stop && iter_->first < coord) {
            iter_ = intv_map_.lower_bound(coord);
        }
    }

    void _append_vertex(coordinate_type pos, coordinate_type coord, bool is_up, bool in_edge) {
        if (!output_.empty() && output_.back().pos() == pos && output_.back().coord() == coord) {
            output_.back().add_edge(is_up, 0, in_edge);
        } else {
            output_.emplace_back(pos, coord, is_up, static_cast<int8_t>(0), in_edge);
        }
    }

    void _process_node(coordinate_type pos, coordinate_type coord,
                       const std::array<CntType, 2> &edges) {
        _update_pos(pos);
        _update_coord(coord);

        // get interval counts in the 4 quandrants about this vertex
        auto stop = intv_map_.end();
        auto pre_cnt1 = (iter_ == intv_map_.begin()) ? CntType{0} : std::prev(iter_, 1)->second;
        auto pre_cnt0 = pre_cnt1 - delta_;
        auto cur_cnt1 = pre_cnt1;
        auto cur_cnt0 = pre_cnt0;
        auto pre_sgn1 = pre_cnt1 > 0;
        auto pre_sgn0 = pre_cnt0 > 0;
        auto cur_sgn1 = pre_sgn1;
        auto cur_sgn0 = pre_sgn0;

        // add down edge to up edge.
        // if this down edge is recorded before, adding it here just cancels the old
        // one out.  If this down edge is not recorded before, that means we have
        // a node with self-cancelled edges before, and the current down edge will
        // annihilate the old down edge and reveal a new up edge.
        auto up_cnt = edges[1] + edges[0];
        auto coord_in_map = (iter_ != stop && iter_->first == coord);
        if (coord_in_map) {
            cur_cnt0 = iter_->second;
            cur_sgn0 = cur_cnt0 > 0;
            // subtract old down edge from up edges
            up_cnt += cur_cnt0 - pre_cnt0;
        }

        // NOTE: debug print statements
        /*
        std::cout << fmt::format("x: {}, y: {}, up_cnt: {}, delta_: {}", coord, pos, up_cnt, delta_)
                  << std::endl;
        */
        if (up_cnt != 0) {
            cur_cnt1 += up_cnt;
            cur_sgn1 = cur_cnt1 > 0;
            // insert or replace interval count
            if (coord_in_map) {
                iter_->second = cur_cnt1;
                ++iter_;
            } else {
                intv_map_.emplace_hint(iter_, coord, cur_cnt1);
            }
        } else if (coord_in_map) {
            // remove down edge
            intv_map_.erase(iter_++);
        }

        // decide whether to record node based on interval count changes
        auto diff_01 = (pre_sgn0 ^ pre_sgn1) || (cur_sgn0 ^ cur_sgn1);
        auto diff_lr = (pre_sgn0 ^ cur_sgn0) || (pre_sgn1 ^ cur_sgn1);
        if ((pre_sgn0 ^ cur_sgn0) && diff_01) {
            _append_vertex(pos, coord, false, pre_sgn0);
        }
        if ((pre_sgn1 ^ cur_sgn1) && diff_01) {
            _append_vertex(pos, coord, true, cur_sgn1);
        }
        if ((pre_sgn0 ^ pre_sgn1) && diff_lr) {
            output_.back().add_h_edge(false, pre_sgn1);
        }
        if ((cur_sgn0 ^ cur_sgn1) && diff_lr) {
            output_.back().add_h_edge(true, cur_sgn0);
        }
        delta_ = cur_cnt1 - cur_cnt0;
    }
};

} // namespace detail_poly90
} // namespace polygon
} // namespace cbag

#endif

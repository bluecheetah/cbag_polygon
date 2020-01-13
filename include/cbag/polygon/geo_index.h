// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
/*
Copyright (c) 2018, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Copyright 2019 Blue Cheetah Analog Design Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef CBAG_POLYGON_GEO_INDEX_H
#define CBAG_POLYGON_GEO_INDEX_H

#include <memory>
#include <utility>
#include <variant>

#include <boost/geometry/index/rtree.hpp>

#include <cbag/polygon/boost_adapt.h>
#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/polygon_data.h>
#include <cbag/polygon/polygon_set_data.h>
#include <cbag/polygon/transformation_util.h>
#include <cbag/util/iterators.h>
#include <cbag/util/overload.h>

namespace cbag {
namespace polygon {
namespace index {

namespace bgi = boost::geometry::index;

template <typename T> class geo_index;

template <typename T> class geo_instance {
  public:
    using coordinate_type = T;
    using box_type = rectangle_data<coordinate_type>;

  private:
    std::shared_ptr<const geo_index<coordinate_type>> master_ = nullptr;
    transformation<coordinate_type> xform_;

  public:
    geo_instance(std::shared_ptr<const geo_index<coordinate_type>> master,
                 transformation<coordinate_type> xform)
        : master_(std::move(master)), xform_(std::move(xform)) {
        if (!master_) {
            throw std::invalid_argument("Cannot create geo_instance with nullptr master.");
        }
    }

    bool operator==(const geo_instance &rhs) const noexcept {
        return master_ == rhs.master_ || xform_ == rhs.xform_;
    }

    const transformation<coordinate_type> &xform() const noexcept { return xform_; }

    bool empty() const;

    auto get_bbox() const -> box_type;

    const polygon_set_data<coordinate_type> &get_master_geometry() const;

    template <typename OutIter>
    void get_intersect(OutIter iter, const box_type &r, coordinate_type spx, coordinate_type spy,
                       bool no_sp, const transformation<coordinate_type> &xform) const;
};

template <typename T> class geo_object {
  public:
    using coordinate_type = T;
    using value_type =
        std::variant<rectangle_data<coordinate_type>, polygon_90_data<coordinate_type>,
                     polygon_45_data<coordinate_type>, polygon_data<coordinate_type>,
                     geo_instance<coordinate_type>>;

  private:
    value_type val_;
    std::array<coordinate_type, 2> sp_{0, 0};

  public:
    template <typename ObjType>
    geo_object(ObjType &&v, coordinate_type spx, coordinate_type spy)
        : val_(std::forward<ObjType>(v)), sp_{spx, spy} {}

    const value_type &value() const noexcept { return val_; }

    std::array<coordinate_type, 2> space() const noexcept { return sp_; }
};

template <typename T> bool operator==(const geo_object<T> &lhs, const geo_object<T> &rhs) {
    return lhs.value() == rhs.value() && lhs.space() == rhs.space();
}

template <typename T> rectangle_data<T> get_bbox(const geo_object<T> &obj) {
    auto ans = std::visit(overload{[](const auto &v) { return cbag::polygon::get_bbox(v); },
                                   [](const geo_instance<T> &v) { return v.get_bbox(); }},
                          obj.value());

    auto [spx, spy] = obj.space();
    return expand(ans, spx, spy);
}

template <typename T> class geo_index {
  public:
    using coordinate_type = T;
    using box_type = rectangle_data<coordinate_type>;
    using tree_value_type = std::pair<box_type, std::size_t>;
    using tree_type = bgi::rtree<tree_value_type, bgi::quadratic<32, 16>>;

  private:
    tree_type index_;
    std::vector<geo_object<coordinate_type>> geo_list_;
    mutable polygon_set_data<T> geo_set_;
    mutable bool cached_ = false;

  public:
    geo_index() = default;

    bool empty() const noexcept { return geo_list_.empty(); }

    box_type get_bbox() const {
        auto tmp = index_.bounds();
        return {tmp.min_corner().template get<0>(), tmp.min_corner().template get<1>(),
                tmp.max_corner().template get<0>(), tmp.max_corner().template get<1>()};
    }

    template <typename ObjType>
    void insert(ObjType &&obj, coordinate_type spx, coordinate_type spy) {
        cached_ = false;
        auto id = geo_list_.size();
        geo_list_.emplace_back(std::forward<ObjType>(obj), spx, spy);
        index_.insert(tree_value_type(cbag::polygon::index::get_bbox(geo_list_.back()), id));
    }

    const polygon_set_data<T> &get_geometry() const {
        if (!cached_) {
            geo_set_.clear();
            geo_set_.reserve(geo_list_.size() * 4);
            for (const auto &lookup : index_) {
                auto &obj = geo_list_[lookup.second];
                std::visit(overload{
                               [this](const auto &v) { geo_set_.insert(v); },
                               [this](const geo_instance<T> &v) {
                                   geo_set_.insert_transform(v.get_master_geometry(), v.xform());
                               },
                           },
                           obj.value());
            }
            cached_ = true;
        }
        return geo_set_;
    }

    void insert(const std::shared_ptr<const geo_index> &master,
                const transformation<coordinate_type> &xform) {
        if (!master->empty()) {
            cached_ = false;
            auto id = geo_list_.size();
            geo_list_.emplace_back(geo_instance<T>(master, xform), 0, 0);
            index_.insert(tree_value_type(cbag::polygon::index::get_bbox(geo_list_.back()), id));
        }
    }

    template <typename OutIter>
    void get_intersect(
        OutIter iter, const box_type &r, coordinate_type spx, coordinate_type spy, bool no_sp,
        const transformation<coordinate_type> &xform = transformation<coordinate_type>()) const {
        auto intersect_box = r;
        if (!no_sp) {
            expand(intersect_box, spx, spy);
        }

        index_.query(
            bgi::intersects(intersect_box),
            cbag::util::lambda_output_iterator(
                [this, &iter, &r, spx, spy, no_sp, xform](const tree_value_type &rhs) {
                    auto cur_sp = geo_list_[rhs.second].space();
                    std::visit(overload{[&iter, &r, spx, spy, no_sp, xform, cur_sp](const auto &v) {
                                            auto test_box = r;
                                            if (!no_sp) {
                                                expand(test_box, std::max(spx, cur_sp[0]),
                                                       std::max(spy, cur_sp[1]));
                                            }
                                            auto pset = polygon_set_data<coordinate_type>();
                                            auto box_set = polygon_set_data<coordinate_type>();
                                            pset.insert(v);
                                            box_set.insert(test_box);
                                            pset &= box_set;
                                            if (!pset.empty())
                                                iter = get_transform(v, xform);
                                        },
                                        [&iter, &r, spx, spy, no_sp,
                                         xform](const geo_instance<coordinate_type> &v) {
                                            v.get_intersect(iter, r, spx, spy, no_sp, xform);
                                        }},
                               geo_list_[rhs.second].value());
                }));
    }
};

template <typename T, typename Fun>
void apply_intersect(const geo_index<T> &obj, Fun lambda, const typename geo_index<T>::box_type &r,
                     T spx, T spy, bool no_sp,
                     const transformation<T> &xform = transformation<T>()) {
    obj.get_intersect(cbag::util::lambda_output_iterator(std::move(lambda)), r, spx, spy, no_sp,
                      xform);
}

template <typename T> bool geo_instance<T>::empty() const { return master_->empty(); }

template <typename T> auto geo_instance<T>::get_bbox() const -> box_type {
    return cbag::polygon::get_transform(master_->get_bbox(), xform_);
}

template <typename T> const polygon_set_data<T> &geo_instance<T>::get_master_geometry() const {
    return master_->get_geometry();
}

template <typename T>
template <typename OutIter>
void geo_instance<T>::get_intersect(OutIter iter, const box_type &r, coordinate_type spx,
                                    coordinate_type spy, bool no_sp,
                                    const transformation<coordinate_type> &xform) const {
    auto test_box = r;
    transform(test_box, -xform_);
    if (swaps_xy(xform_.orient())) {
        std::swap(spx, spy);
    }
    master_->get_intersect(iter, test_box, spx, spy, no_sp, xform_ + xform);
}

} // namespace index
} // namespace polygon
} // namespace cbag

#endif

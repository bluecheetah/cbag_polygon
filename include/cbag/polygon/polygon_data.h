// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_DATA_H
#define CBAG_POLYGON_POLYGON_DATA_H

#include <cbag/polygon/polygon_data_base.h>

namespace cbag {
namespace polygon {

template <typename T> class polygon_data : public polygon_data_base<T, polygon_data<T>> {
  private:
    using base_cls = polygon_data_base<T, polygon_data<T>>;

  public:
    using base_cls::base_cls;
};

template <typename T> struct tag<polygon_data<T>> { using type = polygon_tag; };

template <typename T> struct polygon_traits<polygon_data<T>> {
    using polygon_type = polygon_data<T>;
    using coordinate_type = typename polygon_type::coordinate_type;
    using area_type = typename polygon_type::area_type;
    using iterator_type = typename polygon_type::iterator_type;
    using point_type = typename iterator_type::value_type;

    static iterator_type begin_points(const polygon_type &t) { return t.begin(); }

    static iterator_type end_points(const polygon_type &t) { return t.end(); }

    static std::size_t size(const polygon_type &t) { return t.size(); }

    static void set_points(polygon_type &t, std::vector<point_type> &&data) {
        t.set(std::move(data));
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static void set_points(polygon_type &t, iT start, iT stop, std::size_t n = 3) {
        t.set(start, stop, n);
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static polygon_type construct(iT start, iT stop, std::size_t n = 3) {
        return {start, stop, n};
    }
};

} // namespace polygon
} // namespace cbag

#endif

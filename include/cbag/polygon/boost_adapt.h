// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_BOOST_ADAPT_H
#define CBAG_POLYGON_BOOST_ADAPT_H

#include <utility>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/tags.hpp>

#include <cbag/polygon/point_data.h>
#include <cbag/polygon/rectangle_data.h>

namespace boost {
namespace geometry {
namespace traits {

// adapting point_data class
template <typename T> struct tag<cbag::polygon::point_data<T>> { using type = point_tag; };
template <typename T> struct coordinate_type<cbag::polygon::point_data<T>> { using type = T; };
template <typename T> struct coordinate_system<cbag::polygon::point_data<T>> {
    using type = cs::cartesian;
};
template <typename T> struct dimension<cbag::polygon::point_data<T>> : boost::mpl::int_<2> {};
template <typename T> struct access<cbag::polygon::point_data<T>, 0> {
    using point_type = cbag::polygon::point_data<T>;

    static T get(const point_type &p) { return p[0]; }

    static void set(point_type &p, const T &value) { p[0] = value; }
};

template <typename T> struct access<cbag::polygon::point_data<T>, 1> {
    using point_type = cbag::polygon::point_data<T>;

    static T get(const point_type &p) { return p[1]; }

    static void set(point_type &p, const T &value) { p[1] = value; }
};

// adapting rectangle_data class
template <typename T> struct tag<cbag::polygon::rectangle_data<T>> { using type = box_tag; };

template <typename T> struct point_type<cbag::polygon::rectangle_data<T>> {
    using type = cbag::polygon::point_data<T>;
};

template <typename T> struct indexed_access<cbag::polygon::rectangle_data<T>, min_corner, 0> {
    using box_type = cbag::polygon::rectangle_data<T>;

    static T get(const box_type &b) { return b[0][0]; }

    static void set(box_type &b, const T &value) { b[0][0] = value; }
};

template <typename T> struct indexed_access<cbag::polygon::rectangle_data<T>, min_corner, 1> {
    using box_type = cbag::polygon::rectangle_data<T>;

    static T get(const box_type &b) { return b[1][0]; }

    static void set(box_type &b, const T &value) { b[0][1] = value; }
};

template <typename T> struct indexed_access<cbag::polygon::rectangle_data<T>, max_corner, 0> {
    using box_type = cbag::polygon::rectangle_data<T>;

    static T get(const box_type &b) { return b[0][1]; }

    static void set(box_type &b, const T &value) { b[0][1] = value; }
};

template <typename T> struct indexed_access<cbag::polygon::rectangle_data<T>, max_corner, 1> {
    using box_type = cbag::polygon::rectangle_data<T>;

    static T get(const box_type &b) { return b[1][1]; }

    static void set(box_type &b, const T &value) { b[1][1] = value; }
};

} // namespace traits
} // namespace geometry
} // namespace boost

#endif

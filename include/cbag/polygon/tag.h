// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_TAG_H
#define CBAG_POLYGON_TAG_H

#include <type_traits>

namespace cbag {
namespace polygon {

template <typename T> struct tag {};

struct interval_tag {};

struct point_tag {};

struct polygon_tag {};

struct polygon_45_tag : polygon_tag {};

struct polygon_90_tag : polygon_45_tag {};

struct rectangle_tag : polygon_90_tag {};

struct polygon_set_tag {};

template <typename T> struct interval_traits {};

template <typename T> struct point_traits {};

template <typename T> struct rectangle_traits {};

template <typename T> struct polygon_90_traits {};

template <typename T> struct polygon_45_traits {};

template <typename T> struct polygon_traits {};

template <typename T> using IsInterval = typename interval_traits<T>::coordinate_type;

template <typename T> using IsPoint = typename point_traits<T>::coordinate_type;

template <typename T>
using IsRectangle = typename rectangle_traits<std::decay_t<T>>::coordinate_type;

template <typename T> using IsPoly90 = typename polygon_90_traits<T>::coordinate_type;

template <typename T, typename Tag> struct get_traits {};

template <typename T> struct get_traits<T, interval_tag> { using type = interval_traits<T>; };

template <typename T> struct get_traits<T, point_tag> { using type = point_traits<T>; };

template <typename T> struct get_traits<T, polygon_tag> { using type = polygon_traits<T>; };

template <typename T> struct get_traits<T, polygon_45_tag> { using type = polygon_45_traits<T>; };

template <typename T> struct get_traits<T, polygon_90_tag> { using type = polygon_90_traits<T>; };

template <typename T> struct get_traits<T, rectangle_tag> { using type = rectangle_traits<T>; };

template <typename T> using get_traits_t = typename get_traits<T, typename tag<T>::type>::type;

template <typename T> using coord_type = typename get_traits_t<T>::coordinate_type;

template <typename T> using ucoord_type = std::make_unsigned_t<coord_type<T>>;

template <typename T> using area_type = typename get_traits_t<T>::area_type;

} // namespace polygon
} // namespace cbag

#endif

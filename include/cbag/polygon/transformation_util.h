// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_TRANSFORMATION_UTIL_H
#define CBAG_POLYGON_TRANSFORMATION_UTIL_H

#include <vector>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/interval_data.h>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/polygon_set_data.h>
#include <cbag/polygon/rectangle_concept.h>
#include <cbag/polygon/tag.h>
#include <cbag/polygon/transformation.h>

namespace cbag {
namespace polygon {
namespace dispatch {

template <typename T> T &transform(T &obj, const transformation<coord_type<T>> &xform, point_tag) {
    using traits = get_traits_t<T>;

    auto ans =
        xform.transform(traits::get(obj, orientation_2d::X), traits::get(obj, orientation_2d::Y));

    traits::set(obj, orientation_2d::X, ans[0]);
    traits::set(obj, orientation_2d::Y, ans[1]);

    return obj;
}

template <typename T>
T &transform(T &obj, const transformation<coord_type<T>> &xform, rectangle_tag) {
    using unit = coord_type<T>;

    if (is_valid(obj)) {
        auto ll = point_ll(obj);
        auto ur = point_ur(obj);

        transform(ll, xform, point_tag{});
        transform(ur, xform, point_tag{});
        get_traits_t<T>::set(obj, orientation_2d::X,
                             interval_data<unit>{std::min(ur[0], ll[0]), std::max(ur[0], ll[0])});
        get_traits_t<T>::set(obj, orientation_2d::Y,
                             interval_data<unit>{std::min(ur[1], ll[1]), std::max(ur[1], ll[1])});
    }
    return obj;
}

template <typename T>
T &transform(T &obj, const transformation<coord_type<T>> &xform, polygon_90_tag) {
    auto n = size(obj);
    auto iter = begin_points(obj);
    auto coord_vec = std::vector<coord_type<T>>(n);
    auto reverse = reverse_winding(xform.orient());
    auto stop_idx = n - reverse * (n + 1);
    auto inc = 1 - 2 * reverse;
    for (std::size_t idx = reverse * (n - 1); idx != stop_idx; idx += inc, ++iter) {
        auto tmp = *iter;
        transform(tmp, xform, point_tag{});
        coord_vec[idx] = get(tmp, static_cast<orientation_2d>(idx & 1));
    }
    get_traits_t<T>::set_compact(obj, std::move(coord_vec));
    return obj;
}

template <typename T>
T &transform(T &obj, const transformation<coord_type<T>> &xform, polygon_tag) {
    auto n = size(obj);
    auto iter = begin_points(obj);
    auto pt_vec = std::vector<std::remove_cv_t<std::remove_reference_t<decltype(*iter)>>>(n);
    auto reverse = reverse_winding(xform.orient());
    auto stop_idx = n - reverse * (n + 1);
    auto inc = 1 - 2 * reverse;
    for (std::size_t idx = reverse * (n - 1); idx != stop_idx; idx += inc, ++iter) {
        pt_vec[idx] = *iter;
        transform(pt_vec[idx], xform, point_tag{});
    }
    set_points(obj, std::move(pt_vec));
    return obj;
}

} // namespace dispatch

template <typename T> T &transform(T &obj, const transformation<coord_type<T>> &xform) {
    if (is_null(xform))
        return obj;
    return dispatch::transform(obj, xform, typename tag<T>::type{});
}

template <typename T> T get_transform(T obj, const transformation<coord_type<T>> &xform) {
    dispatch::transform(obj, xform, typename tag<T>::type{});
    return obj;
}

template <typename T>
polygon_set_data<T> &transform(polygon_set_data<T> &obj, const transformation<T> &xform) {
    return obj.transform(xform);
}

} // namespace polygon
} // namespace cbag

#endif

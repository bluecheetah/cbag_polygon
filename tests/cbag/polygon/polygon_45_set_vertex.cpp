// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/polygon_45_set_detail.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/point_data.h>

TEST_CASE("polygon_45_set_vertex operator<", "[polygon_45_set_vertex]") {
    using data_type = std::tuple<std::array<cbag::polygon::point_data<int>, 3>,
                                 std::array<cbag::polygon::point_data<int>, 3>>;
    auto fname = "tests/data/cbag/polygon/polygon_45_set_vertex/less_than.yaml";

    auto [arr1, arr2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto v1 = cbag::polygon::detail_poly45::polygon_45_set_vertex<int, cbag::orientation_2d::X>(
        arr1[0][0], arr1[0][1], arr1[1][0], arr1[1][1], arr1[2][0], arr1[2][1], false);
    auto v2 = cbag::polygon::detail_poly45::polygon_45_set_vertex<int, cbag::orientation_2d::X>(
        arr2[0][0], arr2[0][1], arr2[1][0], arr2[1][1], arr2[2][0], arr2[2][1], false);

    CAPTURE(v1, v2);

    REQUIRE(v1 < v2);
    REQUIRE(!(v2 < v1));
}

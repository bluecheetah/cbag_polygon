// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/point_data.h>
#include <cbag/tests/io.h>

TEST_CASE("point_data getter", "[point_data]") {
    using data_type = std::tuple<int, int>;
    auto fname = "tests/data/cbag/polygon/point_data.yaml";

    auto [x, y] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(x, y);

    auto pt = cbag::polygon::point_data<int>(x, y);

    REQUIRE(pt[0] == x);
    REQUIRE(pt[1] == y);
    REQUIRE(pt[cbag::orientation_2d::X] == x);
    REQUIRE(pt[cbag::orientation_2d::Y] == y);
}

TEST_CASE("point_data structured binding", "[point_data]") {
    using data_type = std::tuple<int, int>;
    auto fname = "tests/data/cbag/polygon/point_data.yaml";

    auto [x, y] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(x, y);

    auto pt = cbag::polygon::point_data<int>(x, y);
    auto &[x2, y2] = pt;

    REQUIRE(x2 == x);
    REQUIRE(y2 == y);
}

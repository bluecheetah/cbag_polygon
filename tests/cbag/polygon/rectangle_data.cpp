// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/rectangle_concept.h>
#include <cbag/polygon/rectangle_data.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/rectangle_data.h>

TEST_CASE("rectangle getter", "[rectangle_data]") {
    using data_type = std::tuple<int, int, int, int>;
    auto fname = "tests/data/cbag/polygon/rectangle_data/getter.yaml";

    auto[xl, yl, xh, yh] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(xl, yl, xh, yh);

    auto rect = cbag::polygon::rectangle_data<int>(xl, yl, xh, yh);

    REQUIRE(cbag::polygon::xl(rect) == xl);
    REQUIRE(cbag::polygon::xh(rect) == xh);
    REQUIRE(cbag::polygon::yl(rect) == yl);
    REQUIRE(cbag::polygon::yh(rect) == yh);
}

TEST_CASE("rectangle::expand", "[rectangle_data]") {
    using c_rect = cbag::polygon::rectangle_data<int>;
    using data_type = std::tuple<c_rect, std::array<int, 2>, c_rect>;
    auto fname = "tests/data/cbag/polygon/rectangle_data/expand.yaml";

    auto[r1, delta, r3] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(r1, delta, r3);

    cbag::polygon::expand(r1, delta[0], delta[1]);
    REQUIRE(r1 == r3);
}

TEST_CASE("rectangle_data convolve_rect", "[rectangle_data]") {
    using c_rect = cbag::polygon::rectangle_data<int>;
    using data_type = std::tuple<c_rect, c_rect, c_rect>;
    auto fname = "tests/data/cbag/polygon/rectangle_data/convolve.yaml";

    auto[r1, r2, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(r1, r2, expect);

    convolve_rect(r1, r2);
    REQUIRE(r1 == expect);
}

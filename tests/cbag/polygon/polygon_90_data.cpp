// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/polygon_90_data.h>
#include <cbag/polygon/polygon_concept.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/point_data.h>
#include <cbag/yaml/rectangle_data.h>

TEST_CASE("polygon_90_data points constructor", "[polygon_90_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::point_data<int>>,
                                 std::vector<cbag::polygon::point_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_90_data/points.yaml";

    auto[pt_vec, expect_vec] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec, expect_vec);

    auto poly = cbag::polygon::polygon_90_data<int>(pt_vec.begin(), pt_vec.end(), pt_vec.size());

    REQUIRE(cbag::polygon::size(poly) == expect_vec.size());

    auto iter1 = cbag::polygon::begin_points(poly);
    auto stop1 = cbag::polygon::end_points(poly);
    for (auto idx = 0; iter1 != stop1; ++iter1, ++idx) {
        REQUIRE(*iter1 == expect_vec[idx]);
    }
}

TEST_CASE("polygon_90_data expand", "[polygon_90_data]") {
    using data_type = std::tuple<std::vector<int>, std::array<int, 2>, std::vector<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_90_data/expand.yaml";

    auto[p1, delta, p2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(p1, delta, p2);

    auto poly1 = cbag::polygon::polygon_90_data<int>(std::move(p1));
    auto poly2 = cbag::polygon::polygon_90_data<int>(std::move(p2));

    cbag::polygon::expand(poly1, delta[0], delta[1]);

    CAPTURE(poly1);

    REQUIRE(poly1 == poly2);
}

TEST_CASE("polygon_90_data is_positive()", "[polygon_90_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::point_data<int>>, bool>;
    auto fname = "tests/data/cbag/polygon/polygon_90_data/is_positive.yaml";

    auto[pt_vec, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec, expect);

    auto poly = cbag::polygon::polygon_90_data<int>(pt_vec.begin(), pt_vec.end(), pt_vec.size());

    CAPTURE(poly);

    REQUIRE(cbag::polygon::is_positive(poly) == expect);
}

TEST_CASE("poly90_convolve_rect_coord_iterator", "[poly90_convolve_rect_coord_iterator]") {
    using data_type =
        std::tuple<std::vector<cbag::polygon::point_data<int>>, cbag::polygon::rectangle_data<int>,
                   std::vector<cbag::polygon::point_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_90_data/convolve_rect.yaml";

    auto[pt_vec, rect, expect_vec] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec, rect, expect_vec);

    auto poly = cbag::polygon::polygon_90_data<int>(pt_vec.begin(), pt_vec.end(), pt_vec.size());
    auto expect = cbag::polygon::polygon_90_data<int>(expect_vec.begin(), expect_vec.end(),
                                                      expect_vec.size());

    CAPTURE(poly, expect);

    auto start = cbag::polygon::poly90_convolve_rect_coord_iterator(
        poly.begin_compact(), poly.end_compact(), xl(rect), yl(rect), xh(rect), yh(rect));
    auto stop = decltype(start)(poly.end_compact());

    auto ans = cbag::polygon::polygon_90_data<int>(start, stop, pt_vec.size());

    REQUIRE(ans == expect);
}

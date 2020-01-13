// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/polygon_data.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/point_data.h>

TEST_CASE("polygon_data points", "[polygon_data]") {
    using data_type = std::vector<cbag::polygon::point_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_data/points.yaml";

    auto pt_vec = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec);

    auto poly = cbag::polygon::polygon_data<int>(pt_vec);

    REQUIRE(cbag::polygon::size(poly) == pt_vec.size());

    auto iter1 = cbag::polygon::begin_points(poly);
    auto stop1 = cbag::polygon::end_points(poly);
    for (auto iter2 = pt_vec.begin(); iter1 != stop1; ++iter1, ++iter2) {
        REQUIRE(*iter1 == *iter2);
    }
}

TEST_CASE("polygon_data is_positive()", "[polygon_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::point_data<int>>, bool>;
    auto fname = "tests/data/cbag/polygon/polygon_data/is_positive.yaml";

    auto [pt_vec, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec, expect);

    auto poly = cbag::polygon::polygon_data<int>(pt_vec.begin(), pt_vec.end(), pt_vec.size());

    CAPTURE(poly);

    REQUIRE(cbag::polygon::is_positive(poly) == expect);
}

TEST_CASE("poly_reduce_iterator", "[polygon_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::point_data<int>>,
                                 std::vector<cbag::polygon::point_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_data/poly_reduce.yaml";

    auto [vec1, vec2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(vec1, vec2);

    auto start = cbag::polygon::dispatch::poly_reduce_iterator(vec1.begin(), vec1.end());

    std::size_t idx = 0;
    for (; start.has_next() && idx < vec2.size(); ++start, ++idx) {
        REQUIRE(*start == vec2[idx]);
    }

    REQUIRE(idx == vec2.size());
    REQUIRE(!start.has_next());
}

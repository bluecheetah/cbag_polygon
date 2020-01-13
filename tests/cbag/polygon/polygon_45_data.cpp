// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/polygon_45_data.h>
#include <cbag/polygon/polygon_concept.h>
#include <cbag/polygon/polygon_set_data.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/point_data.h>
#include <cbag/yaml/polygon_45_data.h>
#include <cbag/yaml/rectangle_data.h>

TEST_CASE("polygon_45_data iterator", "[polygon_45_data]") {
    using data_type = std::vector<cbag::polygon::point_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_45_data/iterator.yaml";

    auto pt_vec = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec);

    auto poly = cbag::polygon::polygon_45_data<int>(pt_vec);

    REQUIRE(cbag::polygon::size(poly) == pt_vec.size());

    auto iter1 = cbag::polygon::begin_points(poly);
    auto stop1 = cbag::polygon::end_points(poly);
    for (auto iter2 = pt_vec.begin(); iter1 != stop1; ++iter1, ++iter2) {
        REQUIRE(*iter1 == *iter2);
    }
}

TEST_CASE("polygon_45_data expand", "[polygon_45_data]") {
    using data_type = std::tuple<cbag::polygon::polygon_45_data<int>, std::array<int, 2>,
                                 cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_45_data/expand.yaml";

    auto[p1, delta, p2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(p1, delta, p2);

    cbag::polygon::expand(p1, delta[0], delta[1]);

    CAPTURE(p1);

    REQUIRE(p1 == p2);
}

TEST_CASE("polygon_45_data area", "[polygon_45_data]") {
    using data_type = std::tuple<cbag::polygon::polygon_45_data<int>, int64_t>;
    auto fname = "tests/data/cbag/polygon/polygon_45_data/area.yaml";

    auto[p1, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(p1, expect);
    REQUIRE(area(p1) == expect);
}

TEST_CASE("poly45_convolve_rect_iterator", "[poly45_convolve_rect_iterator]") {
    using data_type =
        std::tuple<cbag::polygon::polygon_45_data<int>, cbag::polygon::rectangle_data<int>,
                   cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_45_data/convolve_rect.yaml";

    auto[pt_vec, rect, expect_vec] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(pt_vec, rect, expect_vec);

    auto poly = cbag::polygon::polygon_45_data<int>(pt_vec);
    auto expect = cbag::polygon::polygon_45_data<int>(expect_vec);
    CAPTURE(poly, expect);

    auto start = cbag::polygon::poly45_convolve_rect_iterator(poly.begin(), poly.end(), xl(rect),
                                                              yl(rect), xh(rect), yh(rect));
    auto stop = decltype(start)(poly.end());

    auto tmp = cbag::polygon::polygon_45_data<int>(start, stop, pt_vec.size());
    auto set = cbag::polygon::polygon_set_data<int>();
    set.insert(tmp);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    set.get_polygons(std::back_inserter(vec));
    REQUIRE(vec.size() == 1);
    REQUIRE(vec[0] == expect);
}

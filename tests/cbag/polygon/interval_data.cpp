// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/interval_concept.h>
#include <cbag/polygon/interval_data.h>
#include <cbag/tests/io.h>

TEST_CASE("interval getter", "[interval_data]") {
    using data_type = std::tuple<int, int>;
    auto fname = "tests/data/cbag/polygon/interval_data.yaml";

    auto [c0, c1] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(c0, c1);

    auto intv = cbag::polygon::interval_data<int>(c0, c1);

    REQUIRE(intv[0] == c0);
    REQUIRE(intv[1] == c1);
    REQUIRE(cbag::polygon::lower(intv) == c0);
    REQUIRE(cbag::polygon::upper(intv) == c1);
}

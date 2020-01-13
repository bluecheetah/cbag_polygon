// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/polygon_set_data.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/polygon_45_data.h>
#include <cbag/yaml/polygon_90_data.h>
#include <cbag/yaml/rectangle_data.h>

TEST_CASE("polygon_set_data insert simple polygons", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
                                 cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly.yaml";

    auto[poly_list, p2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, p2);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    REQUIRE(vec.size() == 1);
    REQUIRE(vec[0] == p2);
}

TEST_CASE("polygon_set_data insert rectangles", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::rectangle_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_rect.yaml";

    auto[poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    CAPTURE(pset, vec);

    REQUIRE(vec == expect_list);
}

TEST_CASE("polygon_set_data insert polygon90", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_90_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly90.yaml";

    auto[poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);

    REQUIRE(vec == expect_list);
}

TEST_CASE("polygon_set_data insert inverted polygon90", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::rectangle_data<int>, cbag::polygon::polygon_90_data<int>,
                   cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly90_invert.yaml";

    auto[bbox, subtract, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(subtract, expect);
    auto pset = cbag::polygon::polygon_set_data<int>();
    pset.insert(bbox);
    pset.insert(subtract, true);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);

    REQUIRE(vec[0] == expect);
}

TEST_CASE("polygon_set_data insert polygon45 with invert flag", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::polygon_45_data<int>, cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly45_invert.yaml";

    auto[inverted, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(inverted, expect);
    auto pset1 = cbag::polygon::polygon_set_data<int>();
    pset1.insert(inverted, true);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset1.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);
    REQUIRE(vec.size() == 1);
    REQUIRE(vec[0] == expect);
}

TEST_CASE("polygon_set_data insert multiple polygons", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly_multiple.yaml";

    auto[poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);

    REQUIRE(vec == expect_list);
}

TEST_CASE("polygon_set_data clean poly90", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::polygon_90_data<int>, cbag::polygon::polygon_90_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/clean_poly90.yaml";

    auto[poly, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly, expect);

    auto pset1 = cbag::polygon::polygon_set_data<int>();
    auto pset2 = cbag::polygon::polygon_set_data<int>();
    pset1.insert(poly);
    pset2.insert(expect);

    pset1.clean();
    REQUIRE(pset1 == pset2);
}

TEST_CASE("polygon_set_data clean poly45", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::polygon_45_data<int>, cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/clean_poly45.yaml";

    auto[poly, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly, expect);

    auto pset1 = cbag::polygon::polygon_set_data<int>();
    auto pset2 = cbag::polygon::polygon_set_data<int>();
    pset1.insert(poly);
    pset2.insert(expect);

    pset1.clean();
    REQUIRE(pset1 == pset2);
}

TEST_CASE("polygon_set_data get_polygons fracture", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>, bool,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/get_polygons_frac.yaml";

    auto[poly_list, fracture, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, fracture, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec), fracture);

    CAPTURE(vec);

    REQUIRE(vec == expect_list);
}

TEST_CASE("polygon_set_data apply_polygons", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/insert_poly_multiple.yaml";

    auto[poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    auto vec = std::vector<std::string>();
    apply_polygons(pset, [&vec](const auto &v) { vec.emplace_back(to_string(v)); });

    CAPTURE(vec);
    REQUIRE(vec.size() == expect_list.size());

    for (std::size_t idx = 0; idx < vec.size(); ++idx) {
        REQUIRE(vec[idx] == to_string(expect_list[idx]));
    }
}

TEST_CASE("polygon_set_data move_by", "[polygon_set_data]") {
    using data_type = std::tuple<int, int, std::vector<cbag::polygon::polygon_45_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/move.yaml";

    auto[dx, dy, poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(dx, dy, poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);

    pset.move_by(dx, dy);
    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);

    REQUIRE(vec == expect_list);
}

TEST_CASE("polygon_set_data invert", "[polygon_set_data]") {
    using data_type = std::tuple<cbag::polygon::rectangle_data<int>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/invert.yaml";

    auto[bbox, poly_list, expect_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(bbox, poly_list, expect_list);
    auto pset = cbag::polygon::polygon_set_data<int>();
    auto expect = cbag::polygon::polygon_set_data<int>();
    for (const auto &poly : poly_list)
        pset.insert(poly);
    for (const auto &poly : expect_list)
        expect.insert(poly);

    pset.invert(bbox);

    REQUIRE(pset == expect);
}

TEST_CASE("polygon_set_data boolean AND simple", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::polygon_45_data<int>, cbag::polygon::polygon_45_data<int>,
                   cbag::polygon::polygon_45_data<int>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/boolean_and_simple.yaml";

    auto[p1, p2, p3] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(p1, p2, p3);

    auto pset1 = cbag::polygon::polygon_set_data<int>();
    auto pset2 = cbag::polygon::polygon_set_data<int>();
    pset1.insert(p1);
    pset2.insert(p2);

    pset1 &= pset2;

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    pset1.get_polygons(std::back_inserter(vec));

    CAPTURE(vec);

    REQUIRE(vec.size() == 1);
    REQUIRE(vec[0] == p3);
}

TEST_CASE("polygon_set_data boolean SUBTRACT poly90", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_90_data<int>>,
                                 std::vector<cbag::polygon::polygon_90_data<int>>,
                                 std::vector<cbag::polygon::polygon_90_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/boolean_subtract_poly90.yaml";

    auto[vec1, vec2, vec3] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(vec1, vec2, vec3);

    auto pset1 = cbag::polygon::polygon_set_data<int>();
    auto pset2 = cbag::polygon::polygon_set_data<int>();
    auto pset3 = cbag::polygon::polygon_set_data<int>();

    for (const auto &poly : vec1)
        pset1.insert(poly);
    for (const auto &poly : vec2)
        pset2.insert(poly);
    for (const auto &poly : vec3)
        pset3.insert(poly);

    pset1 -= pset2;

    REQUIRE(pset1 == pset3);
}

TEST_CASE("polygon_set_data convolve_rect poly90", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_90_data<int>>,
                                 cbag::polygon::rectangle_data<int>,
                                 std::vector<cbag::polygon::polygon_90_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/convolve_rect_poly90.yaml";

    auto[vec, rect, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(vec, rect, expect);

    auto pset1 = cbag::polygon::polygon_set_data<int>();
    auto pset2 = cbag::polygon::polygon_set_data<int>();

    for (const auto &poly : vec)
        pset1.insert(poly);
    for (const auto &poly : expect)
        pset2.insert(poly);

    pset1.convolve_rect(rect);

    REQUIRE(pset1 == pset2);
}

TEST_CASE("polygon_set_data boolean operations handles negative polygons properly",
          "[polygon_set_data]") {
    auto pset = cbag::polygon::polygon_set_data<int>();
    pset.insert(cbag::polygon::rectangle_data<int>(-50, -10, 50, 10));
    auto expect = pset;

    auto shrink = cbag::polygon::rectangle_data<int>(18, 18, -18, -18);
    auto a = get_convolve_rect(pset, shrink);
    pset |= a;
    REQUIRE(pset == expect);
}

// TODO: enable test case after fixing poly45_convolve_rect_iterator
// TEST_CASE("polygon_set_data convolve_rect poly45", "[polygon_set_data]") {
//     using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
//                                  cbag::polygon::rectangle_data<int>,
//                                  std::vector<cbag::polygon::polygon_45_data<int>>>;
//     auto fname = "tests/data/cbag/polygon/polygon_set_data/convolve_rect_poly45.yaml";

//     auto[vec, rect, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

//     CAPTURE(vec, rect, expect);

//     auto pset1 = cbag::polygon::polygon_set_data<int>();
//     auto pset2 = cbag::polygon::polygon_set_data<int>();

//     for (const auto &poly : vec)
//         pset1.insert(poly);
//     for (const auto &poly : expect)
//         pset2.insert(poly);

//     pset1.convolve_rect(rect);

//     REQUIRE(pset1 == pset2);
// }

TEST_CASE("polygon_set_data rectangle array constructor", "[polygon_set_data]") {
    using data_type =
        std::tuple<cbag::polygon::rectangle_data<int>, std::size_t, std::size_t, int, int>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/rect_array.yaml";

    auto[box, nx, ny, spx, spy] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(box, nx, ny, spx, spy);
    auto expect = cbag::polygon::polygon_set_data<int>();
    for (std::size_t ix = 0; ix < nx; ++ix) {
        for (std::size_t iy = 0; iy < ny; ++iy) {
            expect.insert(get_move_by(box, ix * spx, iy * spy));
        }
    }

    auto test1 =
        cbag::polygon::polygon_set_data<int>(box, cbag::orientation_2d::X, nx, ny, spx, spy);
    REQUIRE(test1 == expect);

    auto test2 =
        cbag::polygon::polygon_set_data<int>(box, cbag::orientation_2d::Y, ny, nx, spy, spx);
    REQUIRE(test2 == expect);
}

TEST_CASE("polygon_set_data get_polygons return poly45", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<std::tuple<int, int, std::string, std::string>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/get_polygons_45.yaml";

    auto[vertex_vec, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto tmp = std::vector<cbag::polygon::detail_poly45::polygon_45_set_vertex<int>>();
    tmp.reserve(vertex_vec.size());
    for (const auto & [ x, y, et, ei ] : vertex_vec) {
        tmp.emplace_back(x, y, static_cast<uint8_t>(std::stoi(et.substr(2), nullptr, 2)),
                         static_cast<uint8_t>(std::stoi(ei.substr(2), nullptr, 2)));
    }

    auto data =
        cbag::polygon::polygon_set_data<int>(cbag::polygon_set_type::POLY90, std::move(tmp));

    auto vec = std::vector<cbag::polygon::polygon_45_data<int>>();
    data.get_polygons(std::back_inserter(vec));

    REQUIRE(vec == expect);
}

TEST_CASE("polygon_set_data get_polygons return poly90", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<std::tuple<int, int, std::string, std::string>>,
                                 std::vector<cbag::polygon::polygon_90_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/get_polygons_90.yaml";

    auto[vertex_vec, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto tmp = std::vector<cbag::polygon::detail_poly45::polygon_45_set_vertex<int>>();
    tmp.reserve(vertex_vec.size());
    for (const auto & [ x, y, et, ei ] : vertex_vec) {
        tmp.emplace_back(x, y, static_cast<uint8_t>(std::stoi(et.substr(2), nullptr, 2)),
                         static_cast<uint8_t>(std::stoi(ei.substr(2), nullptr, 2)));
    }

    auto data =
        cbag::polygon::polygon_set_data<int>(cbag::polygon_set_type::POLY90, std::move(tmp));

    auto vec = std::vector<cbag::polygon::polygon_90_data<int>>();
    auto iter = std::back_inserter(vec);
    data.get_polygons<decltype(iter), cbag::polygon::polygon_90_data<int>>(std::move(iter));

    REQUIRE(vec == expect);
}

TEST_CASE("get_holes", "[polygon_set_data]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
                                 std::vector<cbag::polygon::polygon_45_data<int>>>;
    auto fname = "tests/data/cbag/polygon/polygon_set_data/get_holes.yaml";

    auto[polygons, holes] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto tmp = cbag::polygon::polygon_set_data<int>();
    for (const auto &p : polygons)
        tmp.insert(p);
    auto expect = cbag::polygon::polygon_set_data<int>();
    for (const auto &p : holes) {
        tmp.insert(p, true);
        expect.insert(p);
    }

    auto ans = get_holes(tmp);
    REQUIRE(ans == expect);
}

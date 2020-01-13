// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/geo_index.h>
#include <cbag/polygon/polygon_set_data.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/polygon_45_data.h>
#include <cbag/yaml/rectangle_data.h>
#include <cbag/yaml/transformation.h>

TEST_CASE("geo_index insert rectangles", "[geo_index]") {
    using data_type = std::vector<cbag::polygon::rectangle_data<int>>;
    auto fname = "tests/data/cbag/polygon/geo_index/insert_rect.yaml";

    auto rect_list = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(rect_list);

    auto index = cbag::polygon::index::geo_index<int>();
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &obj : rect_list) {
        index.insert(obj, 0, 0);
        pset.insert(obj);
    }

    REQUIRE(pset == index.get_geometry());

    auto box = get_bbox(pset);
    if (cbag::polygon::is_valid(box))
        REQUIRE(index.get_bbox() == box);
    else
        REQUIRE(!cbag::polygon::is_valid(box));
}

TEST_CASE("geo_index insert polygons", "[geo_index]") {
    using data_type = std::vector<cbag::polygon::polygon_45_data<int>>;

    auto fname = "tests/data/cbag/polygon/geo_index/insert_poly45.yaml";

    auto poly_list = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list);

    auto index = cbag::polygon::index::geo_index<int>();
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &obj : poly_list) {
        index.insert(obj, 0, 0);
        pset.insert(obj);
    }

    REQUIRE(pset == index.get_geometry());

    auto box = get_bbox(pset);
    if (cbag::polygon::is_valid(box))
        REQUIRE(index.get_bbox() == box);
    else
        REQUIRE(!cbag::polygon::is_valid(box));
}

TEST_CASE("geo_index insert_instance", "[geo_index]") {
    using data_type = std::tuple<std::vector<cbag::polygon::polygon_45_data<int>>,
                                 cbag::polygon::transformation<int>>;

    auto fname = "tests/data/cbag/polygon/geo_index/insert_inst.yaml";

    auto[poly_list, xform] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(poly_list, xform);

    auto index0 = std::make_shared<cbag::polygon::index::geo_index<int>>();
    auto pset = cbag::polygon::polygon_set_data<int>();
    for (const auto &obj : poly_list) {
        index0->insert(obj, 0, 0);
        pset.insert(obj);
    }

    auto index = cbag::polygon::index::geo_index<int>();
    index.insert(index0, xform);
    pset.transform(xform);

    auto box = get_bbox(pset);
    if (cbag::polygon::is_valid(box))
        REQUIRE(index.get_bbox() == box);
    else
        REQUIRE(!cbag::polygon::is_valid(box));
}

TEST_CASE("geo_index get_intersect", "[geo_index]") {
    using data_type =
        std::tuple<std::vector<std::tuple<cbag::polygon::polygon_45_data<int>, int, int>>,
                   cbag::polygon::transformation<int>,
                   std::vector<std::tuple<cbag::polygon::polygon_45_data<int>, int, int>>,
                   cbag::polygon::rectangle_data<int>, int, int, bool,
                   std::vector<cbag::polygon::polygon_45_data<int>>>;

    auto fname = "tests/data/cbag/polygon/geo_index/get_intersect.yaml";

    auto[poly_list, inst_xform, inst_poly_list, box, spx, spy, no_sp, expect_list] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(inst_xform, box, spx, spy, no_sp, expect_list);

    auto index = cbag::polygon::index::geo_index<int>();
    for (const auto[obj, obj_spx, obj_spy] : poly_list) {
        index.insert(obj, obj_spx, obj_spy);
    }

    auto inst_index = std::make_shared<cbag::polygon::index::geo_index<int>>();
    for (const auto[obj, obj_spx, obj_spy] : inst_poly_list) {
        inst_index->insert(obj, obj_spx, obj_spy);
    }
    index.insert(inst_index, inst_xform);

    std::vector<
        std::variant<cbag::polygon::rectangle_data<int>, cbag::polygon::polygon_90_data<int>,
                     cbag::polygon::polygon_45_data<int>, cbag::polygon::polygon_data<int>>>
        vec;
    index.get_intersect(std::back_inserter(vec), box, spx, spy, no_sp);

    REQUIRE(vec.size() == expect_list.size());

    for (std::size_t idx = 0; idx < vec.size(); ++idx) {
        auto val_ptr = std::get_if<cbag::polygon::polygon_45_data<int>>(&(vec[idx]));
        REQUIRE(val_ptr != nullptr);
        REQUIRE(*val_ptr == expect_list[idx]);
    }
}

TEST_CASE("geo_index apply_intersect", "[geo_index]") {
    using data_type =
        std::tuple<std::vector<std::tuple<cbag::polygon::polygon_45_data<int>, int, int>>,
                   cbag::polygon::transformation<int>,
                   std::vector<std::tuple<cbag::polygon::polygon_45_data<int>, int, int>>,
                   cbag::polygon::rectangle_data<int>, int, int, bool,
                   std::vector<cbag::polygon::polygon_45_data<int>>>;

    auto fname = "tests/data/cbag/polygon/geo_index/get_intersect.yaml";

    auto[poly_list, inst_xform, inst_poly_list, box, spx, spy, no_sp, expect_list] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(inst_xform, box, spx, spy, no_sp, expect_list);

    auto index = cbag::polygon::index::geo_index<int>();
    for (const auto[obj, obj_spx, obj_spy] : poly_list) {
        index.insert(obj, obj_spx, obj_spy);
    }

    auto inst_index = std::make_shared<cbag::polygon::index::geo_index<int>>();
    for (const auto[obj, obj_spx, obj_spy] : inst_poly_list) {
        inst_index->insert(obj, obj_spx, obj_spy);
    }
    index.insert(inst_index, inst_xform);

    std::vector<std::string> vec;
    apply_intersect(index, [&vec](const auto &v) { vec.emplace_back(cbag::polygon::to_string(v)); },
                    box, spx, spy, no_sp);

    REQUIRE(vec.size() == expect_list.size());

    for (std::size_t idx = 0; idx < vec.size(); ++idx) {
        REQUIRE(vec[idx] == to_string(expect_list[idx]));
    }
}

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/polygon_set_data.h>
#include <cbag/polygon/transformation_util.h>
#include <cbag/tests/io.h>
#include <cbag/yaml/point_data.h>
#include <cbag/yaml/polygon_90_data.h>
#include <cbag/yaml/rectangle_data.h>

std::array<std::array<int, 2>, 2> get_matrix(cbag::orientation orient) {
    auto ans =
        std::array<std::array<int, 2>, 2>{std::array<int, 2>{1, 0}, std::array<int, 2>{0, 1}};

    auto code = static_cast<int>(orient);
    if (code >> 2) {
        std::swap(ans[0], ans[1]);
    }
    if (code & 1) {
        ans[0] = {-ans[0][0], -ans[0][1]};
    }
    if ((code >> 1) & 1) {
        ans[1] = {-ans[1][0], -ans[1][1]};
    }
    return ans;
}

std::array<std::array<int, 2>, 2> invert_matrix(std::array<std::array<int, 2>, 2> m) {
    auto scale = (m[0][0] * m[1][1] - m[1][0] * m[0][1]);
    auto tmp = m[0][0];
    m[0][0] = scale * m[1][1];
    m[1][1] = scale * tmp;
    m[1][0] *= -scale;
    m[0][1] *= -scale;
    return m;
}

std::array<std::array<int, 2>, 2> mult_matrix(const std::array<std::array<int, 2>, 2> &a,
                                              const std::array<std::array<int, 2>, 2> &b) {
    std::array<std::array<int, 2>, 2> ans;
    ans[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
    ans[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
    ans[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
    ans[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];
    return ans;
}

std::array<int, 2> mult_point(const std::array<std::array<int, 2>, 2> &a, int x, int y) {
    std::array<int, 2> ans;
    ans[0] = a[0][0] * x + a[0][1] * y;
    ans[1] = a[1][0] * x + a[1][1] * y;
    return ans;
}

TEST_CASE("invert orientation", "[orientation]") {
    for (int code = 0; code < 8; ++code) {
        auto orient = static_cast<cbag::orientation>(code);
        auto inverse = -orient;
        auto m1 = get_matrix(orient);
        auto m2 = get_matrix(inverse);

        CAPTURE(code, inverse);
        REQUIRE(m2 == invert_matrix(m1));
    }
}

TEST_CASE("concat orientation", "[orientation]") {
    for (int code1 = 0; code1 < 8; ++code1) {
        auto orient1 = static_cast<cbag::orientation>(code1);
        auto m1 = get_matrix(orient1);
        for (int code2 = 0; code2 < 8; ++code2) {
            auto orient2 = static_cast<cbag::orientation>(code2);
            auto m2 = get_matrix(orient2);

            auto concat = orient1 + orient2;
            auto expect = mult_matrix(m2, m1);
            auto prod = get_matrix(concat);

            CAPTURE(m1, m2);
            REQUIRE(prod == expect);
        }
    }
}

TEST_CASE("orientation name defined properly", "[orientation]") {
    // make sure R90 is 90 degree counter-clockwise
    auto mat = get_matrix(cbag::orientation::R90);
    REQUIRE(mat[0][0] == 0);
    REQUIRE(mat[0][1] == -1);
    REQUIRE(mat[1][0] == 1);
    REQUIRE(mat[1][1] == 0);
    REQUIRE(cbag::orientation::MXR90 == cbag::orientation::MX + cbag::orientation::R90);
    REQUIRE(cbag::orientation::MYR90 == cbag::orientation::MY + cbag::orientation::R90);
}

TEST_CASE("transform point", "[transformation]") {
    using data_type = std::tuple<int, int, int, int>;
    auto fname = "tests/data/cbag/polygon/transformation/transform.yaml";

    auto [x, y, x1, y1] = GENERATE_COPY(read_test_vector<data_type>(fname));

    for (int code = 0; code < 8; ++code) {
        CAPTURE(code, x, y, x1, y1);
        auto orient = static_cast<cbag::orientation>(code);
        auto xform = cbag::polygon::transformation(x1, y1, orient);

        auto m = get_matrix(orient);
        auto expect = mult_point(m, x, y);
        auto pt = xform.transform(x, y);

        // test transformation
        REQUIRE(pt[0] == expect[0] + x1);
        REQUIRE(pt[1] == expect[1] + y1);
    }
}

TEST_CASE("inverse transform", "[transformation]") {
    using data_type = std::tuple<int, int, int, int>;
    auto fname = "tests/data/cbag/polygon/transformation/transform.yaml";

    auto [x, y, x1, y1] = GENERATE_COPY(read_test_vector<data_type>(fname));

    for (int code = 0; code < 8; ++code) {
        CAPTURE(code, x, y, x1, y1);

        auto orient = static_cast<cbag::orientation>(code);
        auto xform = cbag::polygon::transformation(x1, y1, orient);
        auto inverse = -xform;

        auto pt = xform.transform(x, y);
        pt = inverse.transform(pt[0], pt[1]);

        REQUIRE(pt[0] == x);
        REQUIRE(pt[1] == y);
    }
}

TEST_CASE("concat transform", "[transformation]") {
    using data_type = std::tuple<int, int, int, int>;
    auto fname = "tests/data/cbag/polygon/transformation/transform.yaml";

    auto [x1, y1, x2, y2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    for (int code1 = 0; code1 < 8; ++code1) {
        auto orient1 = static_cast<cbag::orientation>(code1);
        auto xform1 = cbag::polygon::transformation(x1, y1, orient1);

        for (int code2 = 0; code2 < 8; ++code2) {
            CAPTURE(code1, code2, x1, y1, x2, y2);

            auto orient2 = static_cast<cbag::orientation>(code2);
            auto xform2 = cbag::polygon::transformation(x2, y2, orient2);

            auto xform3 = xform1 + xform2;

            auto tmp = xform1.transform(x1, y2);
            auto test = xform2.transform(tmp[0], tmp[1]);
            auto expect = xform3.transform(x1, y2);

            REQUIRE(test == expect);
        }
    }
}

TEST_CASE("transform rect", "[transformation]") {
    using value_type = cbag::polygon::rectangle_data<int>;
    using data_type = std::tuple<std::array<int, 3>, value_type, value_type>;
    auto fname = "tests/data/cbag/polygon/transformation/transform_rect.yaml";

    auto [xform_info, obj, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(xform_info, obj, expect);

    auto orient = static_cast<cbag::orientation>(xform_info[0]);
    auto xform = cbag::polygon::transformation(xform_info[1], xform_info[2], orient);

    auto ans = obj;
    cbag::polygon::transform(ans, xform);

    REQUIRE(ans == expect);
}

TEST_CASE("transform poly90", "[transformation]") {
    using value_type = cbag::polygon::polygon_90_data<int>;
    using data_type = std::tuple<std::array<int, 3>, value_type, value_type>;
    auto fname = "tests/data/cbag/polygon/transformation/transform_poly90.yaml";

    auto [xform_info, obj, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto orient = static_cast<cbag::orientation>(xform_info[0]);
    auto xform = cbag::polygon::transformation(xform_info[1], xform_info[2], orient);

    auto ans = obj;
    cbag::polygon::transform(ans, xform);

    CAPTURE(xform_info, obj, ans, expect);

    REQUIRE(ans == expect);
}

TEST_CASE("transform polygon_45_set_vertex", "[transformation]") {
    using point_type = cbag::polygon::point_data<int>;
    using vertex_type = cbag::polygon::detail_poly45::polygon_45_set_vertex<int>;
    using data_type =
        std::tuple<std::array<int, 3>, std::vector<point_type>, std::vector<point_type>>;
    auto fname = "tests/data/cbag/polygon/transformation/polygon_45_set_vertex.yaml";

    auto [xform_info, pts1, pts2] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto orient = static_cast<cbag::orientation>(xform_info[0]);
    auto xform = cbag::polygon::transformation(xform_info[1], xform_info[2], orient);

    auto v1 =
        vertex_type(pts1[0][0], pts1[0][1], pts1[1][0], pts1[1][1], pts1[2][0], pts1[2][1], false);
    auto v2 =
        vertex_type(pts2[0][0], pts2[0][1], pts2[1][0], pts2[1][1], pts2[2][0], pts2[2][1], false);

    CAPTURE(xform_info, v1, v2);

    v1.transform(xform);

    REQUIRE(v1 == v2);
}

TEST_CASE("transform polygon_set_data with poly90", "[transformation]") {
    using value_type = cbag::polygon::polygon_90_data<int>;
    using data_type = std::tuple<std::array<int, 3>, value_type, value_type>;
    auto fname = "tests/data/cbag/polygon/transformation/transform_polygon_set_90.yaml";

    auto [xform_info, obj, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto orient = static_cast<cbag::orientation>(xform_info[0]);
    auto xform = cbag::polygon::transformation(xform_info[1], xform_info[2], orient);

    CAPTURE(xform_info, obj, expect);

    auto set1 = cbag::polygon::polygon_set_data<int>();
    auto set2 = cbag::polygon::polygon_set_data<int>();
    set1.insert(obj);
    set2.insert(expect);
    cbag::polygon::transform(set1, xform);

    REQUIRE(set1 == set2);
}

TEST_CASE("transform polygon_set_data with poly90 after clean", "[transformation]") {
    using value_type = cbag::polygon::polygon_90_data<int>;
    using data_type = std::tuple<std::array<int, 3>, value_type, value_type>;
    auto fname = "tests/data/cbag/polygon/transformation/transform_polygon_set_90.yaml";

    auto [xform_info, obj, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto orient = static_cast<cbag::orientation>(xform_info[0]);
    auto xform = cbag::polygon::transformation(xform_info[1], xform_info[2], orient);

    CAPTURE(xform_info, obj, expect);

    auto set1 = cbag::polygon::polygon_set_data<int>();
    auto set2 = cbag::polygon::polygon_set_data<int>();
    set1.insert(obj);
    set2.insert(expect);

    set1.clean();
    cbag::polygon::transform(set1, xform);

    REQUIRE(set1 == set2);
}

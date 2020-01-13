// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_POLYGON_DATA_BASE_H
#define CBAG_YAML_POLYGON_DATA_BASE_H

#include <yaml-cpp/yaml.h>

#include <cbag/yaml/point_data.h>

namespace YAML {

template <typename S> struct convert_poly {
    using value_type = S;

    static Node encode(const value_type &rhs) {
        auto root = Node(NodeType::Sequence);
        for (const auto &pt : rhs) {
            root.push_back(pt);
        }
        return root;
    }

    static bool decode(const Node &node, value_type &rhs) {
        if (!node.IsSequence()) {
            return false;
        }
        try {
            rhs.set(node.as<std::vector<typename value_type::point_type>>());
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace YAML

#endif

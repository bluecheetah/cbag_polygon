// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_POLYGON_90_DATA_H
#define CBAG_YAML_POLYGON_90_DATA_H

#include <yaml-cpp/yaml.h>

#include <cbag/polygon/polygon_90_data.h>

namespace YAML {

template <typename T> struct convert<cbag::polygon::polygon_90_data<T>> {
    using value_type = cbag::polygon::polygon_90_data<T>;

    static Node encode(const value_type &rhs) {
        auto root = Node(NodeType::Sequence);
        auto stop = rhs.end_compact();
        for (auto iter = rhs.begin_compact(); iter != stop; ++iter) {
            root.push_back(*iter);
        }
        return root;
    }

    static bool decode(const Node &node, value_type &rhs) {
        if (!node.IsSequence()) {
            return false;
        }
        try {
            rhs = node.as<std::vector<T>>();
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace YAML

#endif

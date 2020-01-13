// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_POINT_DATA_H
#define CBAG_YAML_POINT_DATA_H

#include <yaml-cpp/yaml.h>

#include <cbag/polygon/point_data.h>

namespace YAML {

template <typename T> struct convert<cbag::polygon::point_data<T>> {
    using value_type = cbag::polygon::point_data<T>;

    static Node encode(const value_type &rhs) {
        auto root = Node(NodeType::Sequence);
        root.push_back(rhs[0]);
        root.push_back(rhs[1]);
        return root;
    }

    static bool decode(const Node &node, value_type &rhs) {
        if (!node.IsSequence() || node.size() != 2) {
            return false;
        }
        try {
            rhs[0] = node[0].as<T>();
            rhs[1] = node[1].as<T>();
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace YAML

#endif

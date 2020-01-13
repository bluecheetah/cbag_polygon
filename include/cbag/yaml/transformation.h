// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_TRANSFORMATION_H
#define CBAG_YAML_TRANSFORMATION_H

#include <yaml-cpp/yaml.h>

#include <cbag/polygon/transformation.h>
#include <cbag/yaml/orientation.h>

namespace YAML {

template <typename T> struct convert<cbag::polygon::transformation<T>> {
    using value_type = cbag::polygon::transformation<T>;

    static Node encode(const value_type &rhs) {
        auto root = Node(NodeType::Sequence);
        root.push_back(x(rhs));
        root.push_back(y(rhs));
        root.push_back(rhs.orient());
        return root;
    }

    static bool decode(const Node &node, value_type &rhs) {
        if (!node.IsSequence() || node.size() != 3) {
            return false;
        }
        try {
            rhs = value_type(node[0].as<T>(), node[1].as<T>(), node[2].as<cbag::orientation>());
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace YAML

#endif

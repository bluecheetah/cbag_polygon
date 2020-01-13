// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_ORIENTATION_H
#define CBAG_YAML_ORIENTATION_H

#include <yaml-cpp/yaml.h>

#include <cbag/polygon/enum.h>

namespace YAML {

template <> struct convert<cbag::orientation> {
    using value_type = cbag::orientation;

    static Node encode(const value_type &rhs) {
        auto root = Node(NodeType::Scalar);
        root = cbag::to_string(rhs);
        return root;
    }

    static bool decode(const Node &node, value_type &rhs) {
        if (!node.IsScalar()) {
            return false;
        }
        try {
            rhs = cbag::to_orientation(node.as<std::string>());
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace YAML

#endif

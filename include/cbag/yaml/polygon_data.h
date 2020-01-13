// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_YAML_POLYGON_DATA_H
#define CBAG_YAML_POLYGON_DATA_H

#include <yaml-cpp/yaml.h>

#include <cbag/polygon/polygon_data.h>
#include <cbag/yaml/polygon_data_base.h>

namespace YAML {

template <typename T>
struct convert<cbag::polygon::polygon_data<T>>
    : public convert_poly<cbag::polygon::polygon_data<T>> {};

} // namespace YAML

#endif

// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_TESTS_IO_H
#define CBAG_TESTS_IO_H

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include <yaml-cpp/yaml.h>

inline std::string read_file(const std::string &fname) {
    std::ifstream in(fname);
    std::stringstream stream;
    stream << in.rdbuf();
    return stream.str();
}

template <typename T> class yaml_generator : public Catch::Generators::IGenerator<T> {
  private:
    std::vector<T> data_;
    std::size_t idx_;

  public:
    yaml_generator(const std::string &fname) : idx_(0) {
        auto node = YAML::LoadFile(fname);
        data_ = node.as<std::vector<T>>();
    }

    const T &get() const override { return data_[idx_]; }
    bool next() override { return (++idx_) < data_.size(); }
};

template <typename T>
Catch::Generators::GeneratorWrapper<T> read_test_vector(const std::string &fname) {
    return Catch::Generators::GeneratorWrapper<T>(
        std::unique_ptr<Catch::Generators::IGenerator<T>>(new yaml_generator<T>(fname)));
}

#endif

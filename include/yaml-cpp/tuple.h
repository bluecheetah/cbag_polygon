// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
/*
Copyright (c) 2018, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Copyright 2019 Blue Cheetah Analog Design Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef YAML_CPP_TUPLE_H
#define YAML_CPP_TUPLE_H

#include <tuple>
#include <type_traits>

#include <yaml-cpp/yaml.h>

// code to iterate over tuple, from:
// https://stackoverflow.com/questions/26902633/how-to-iterate-over-a-stdtuple-in-c-11/26902803
template <class F, class... Ts, std::size_t... Is>
void for_each_in_tuple(const std::tuple<Ts...> &tuple, F func, std::index_sequence<Is...>) {
    using expander = int[];
    (void)expander{0, ((void)func(std::get<Is>(tuple)), 0)...};
}

template <class F, class... Ts, std::size_t... Is>
void set_each_in_tuple(std::tuple<Ts...> &tuple, F func, std::index_sequence<Is...>) {
    using expander = int[];
    (void)expander{0, (std::get<Is>(tuple) = func(Is).template as<Ts>(), 0)...};
}

// YAML conversion methods for std::unordered_map
namespace YAML {
template <typename... Args> struct convert<std::tuple<Args...>> {
    static Node encode(const std::tuple<Args...> &rhs) {
        Node root(NodeType::Sequence);
        for_each_in_tuple(
            rhs, [&root](const auto &x) { root.push_back(x); },
            std::make_index_sequence<sizeof...(Args)>());
        return root;
    }

    static bool decode(const Node &node, std::tuple<Args...> &rhs) {
        if (!node.IsSequence() ||
            node.size() != std::tuple_size_v<std::remove_reference_t<decltype(rhs)>>)
            return false;

        set_each_in_tuple(
            rhs, [&node](std::size_t idx) { return node[idx]; },
            std::make_index_sequence<sizeof...(Args)>());
        return true;
    }
};

} // namespace YAML

#endif

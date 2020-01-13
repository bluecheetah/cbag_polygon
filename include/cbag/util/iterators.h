// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_UTIL_ITERATORS_H
#define CBAG_UTIL_ITERATORS_H

#include <algorithm>
#include <functional>
#include <vector>

namespace cbag {
namespace util {

template <typename F> class lambda_output_iterator {
  public:
    using iterator_category = std::output_iterator_tag;
    using value_type = void;
    using difference_type = void;
    using pointer = void;
    using reference = void;

  private:
    F fun_;

  public:
    explicit lambda_output_iterator(F &&lambda) : fun_(std::move(lambda)) {}

    lambda_output_iterator &operator*() noexcept { return *this; }

    lambda_output_iterator &operator++() noexcept { return *this; }

    template <typename V> lambda_output_iterator &operator=(V &&value) {
        fun_(std::forward<V>(value));
        return *this;
    }
};

template <typename T> class min_heap_iterator {

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = const T *;
    using reference = const T &;

  private:
    std::vector<T> *ptr_ = nullptr;
    std::greater<T> comp_;
    std::size_t remain_ = 0;

  public:
    min_heap_iterator() = default;

    min_heap_iterator(std::vector<T> &data, bool is_end)
        : ptr_(&data), remain_(is_end ? 0 : data.size()) {}

    bool operator==(const min_heap_iterator &rhs) const noexcept { return remain_ == rhs.remain_; }

    bool operator!=(const min_heap_iterator &rhs) const noexcept { return !(*this == rhs); }

    reference operator*() const { return (*ptr_)[0]; }

    pointer operator->() const { return &((*ptr_)[0]); }

    min_heap_iterator &operator++() {
        std::pop_heap(ptr_->begin(), ptr_->begin() + remain_, comp_);
        --remain_;
        return *this;
    }
};

} // namespace util
} // namespace cbag

#endif

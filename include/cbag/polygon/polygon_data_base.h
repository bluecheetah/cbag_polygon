// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_DATA_BASE_H
#define CBAG_POLYGON_POLYGON_DATA_BASE_H

#include <vector>

#include <cbag/polygon/point_data.h>

namespace cbag {
namespace polygon {

template <typename T, typename Derived> class polygon_data_base {
  public:
    using point_type = point_data<T>;
    using coordinate_type = typename point_type::coordinate_type;
    using iterator_type = typename std::vector<point_type>::const_iterator;
    using area_type = typename coordinate_traits<coordinate_type>::area_type;

  private:
    std::vector<point_type> pts_;

  public:
    polygon_data_base() = default;

    explicit polygon_data_base(std::vector<point_type> data) : pts_(std::move(data)) {
        if (pts_.size() < 3)
            pts_.clear();
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    polygon_data_base(iT start, iT stop, std::size_t n = 3) {
        set(start, stop, n);
    }

    bool operator==(const polygon_data_base &rhs) const { return pts_ == rhs.pts_; }

    auto begin() const noexcept -> iterator_type { return pts_.begin(); }

    auto end() const noexcept -> iterator_type { return pts_.end(); }

    std::size_t size() const noexcept { return pts_.size(); }

    Derived &set(std::vector<point_type> &&data) {
        pts_ = std::move(data);
        if (pts_.size() < 3)
            pts_.clear();

        return static_cast<Derived &>(*this);
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    Derived &set(iT start, iT stop, std::size_t n = 3) {
        pts_.clear();
        pts_.reserve(n);
        for (; start != stop; ++start) {
            pts_.emplace_back(x(*start), y(*start));
        }
        if (pts_.size() < 3)
            pts_.clear();
        return static_cast<Derived &>(*this);
    }
};

template <typename T, typename D>
bool operator!=(const polygon_data_base<T, D> &lhs, const polygon_data_base<T, D> &rhs) {
    return !(lhs == rhs);
}

template <typename T, typename D> std::string to_string(const polygon_data_base<T, D> &obj) {
    std::string ans;
    auto n = obj.size();
    if (n < 3)
        ans = "[]";
    else {
        ans = "[";
        auto iter = obj.begin();
        auto stop = obj.end();
        ans += to_string(*iter);
        ++iter;
        for (; iter != stop; ++iter) {
            ans += ", ";
            ans += to_string(*iter);
        }
        ans += "]";
    }
    return ans;
}

template <typename T, typename D>
std::ostream &operator<<(std::ostream &stream, const polygon_data_base<T, D> &obj) {
    stream << to_string(obj);
    return stream;
}

} // namespace polygon
} // namespace cbag

#endif

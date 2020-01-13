// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_POLYGON_90_DATA_H
#define CBAG_POLYGON_POLYGON_90_DATA_H

#include <type_traits>
#include <vector>

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/point_data.h>
#include <cbag/polygon/polygon_90_concept.h>
#include <cbag/polygon/polygon_concept.h>

namespace cbag {
namespace polygon {

template <typename T> class polygon_90_data {
  public:
    using coordinate_type = T;
    using compact_iterator_type = typename std::vector<coordinate_type>::const_iterator;
    using area_type = typename coordinate_traits<coordinate_type>::area_type;
    using iterator_type = poly90_point_iterator<compact_iterator_type, coordinate_type>;
    using point_type = typename iterator_type::value_type;

  private:
    std::vector<coordinate_type> coords_;

  public:
    polygon_90_data() = default;

    explicit polygon_90_data(std::vector<coordinate_type> compact_data)
        : coords_(std::move(compact_data)) {
        _fix_coord();
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    polygon_90_data(iT start, iT stop, std::size_t n = 4) {
        set_points(start, stop, n);
    }

    template <typename iT,
              std::enable_if_t<std::is_same_v<coordinate_type, typename iT::value_type>, int> = 0>
    polygon_90_data(iT start_compact, iT stop_compact, std::size_t n = 4) {
        set_compact(start_compact, stop_compact, n);
    }

    bool operator==(const polygon_90_data &rhs) const { return coords_ == rhs.coords_; }

    auto begin() const -> iterator_type { return {coords_.begin(), coords_.end()}; }

    auto end() const -> iterator_type { return iterator_type{coords_.end()}; }

    auto begin_compact() const noexcept -> compact_iterator_type { return coords_.begin(); }

    auto end_compact() const noexcept -> compact_iterator_type { return coords_.end(); }

    std::size_t size() const noexcept { return coords_.size(); }

    polygon_90_data &operator=(std::vector<coordinate_type> data) {
        coords_ = std::move(data);
        _fix_coord();
        return *this;
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    polygon_90_data &set_points(iT start, iT stop, std::size_t n = 4) {
        coords_.clear();
        if (start != stop) {
            coords_.reserve(n);

            auto orient = 0;
            auto y0 = get(*start, orientation_2d::Y);
            for (; start != stop; ++start, orient ^= 1) {
                coords_.emplace_back(get(*start, static_cast<orientation_2d>(orient)));
            }

            n = coords_.size();
            if (n < 4)
                coords_.clear();
            else if (n & 1)
                coords_.emplace_back(y0);
        }

        return *this;
    }

    template <typename iT,
              std::enable_if_t<std::is_same_v<coordinate_type, typename iT::value_type>, int> = 0>
    polygon_90_data &set_compact(iT start_compact, iT stop_compact, std::size_t n = 4) {
        coords_.clear();
        coords_.reserve(n);
        for (; start_compact != stop_compact; ++start_compact) {
            coords_.emplace_back(*start_compact);
        }
        _fix_coord();

        return *this;
    }

  private:
    void _fix_coord() {
        auto n = coords_.size();
        if (n < 4)
            coords_.clear();
        else if (n & 1)
            coords_.pop_back();
    }

}; // namespace polygon

template <typename T>
bool operator!=(const polygon_90_data<T> &lhs, const polygon_90_data<T> &rhs) {
    return !(lhs == rhs);
}

template <typename T> std::string to_string(const polygon_90_data<T> &obj) {
    std::string ans;
    auto n = obj.size();
    if (n < 4) {
        ans = "[]";
    } else {
        ans += "[";
        auto iter = obj.begin_compact();
        auto stop = obj.end_compact();
        ans += std::to_string(*iter);
        ++iter;
        for (; iter != stop; ++iter) {
            ans += ", ";
            ans += std::to_string(*iter);
        }
        ans += "]";
    }
    return ans;
}

template <typename T>
std::ostream &operator<<(std::ostream &stream, const polygon_90_data<T> &obj) {
    stream << to_string(obj);
    return stream;
}

template <typename T> struct tag<polygon_90_data<T>> { using type = polygon_90_tag; };

template <typename T> struct polygon_90_traits<polygon_90_data<T>> {
    using polygon_type = polygon_90_data<T>;
    using coordinate_type = typename polygon_type::coordinate_type;
    using area_type = typename polygon_type::area_type;
    using compact_iterator_type = typename polygon_type::compact_iterator_type;
    using iterator_type = typename polygon_type::iterator_type;
    using point_type = typename iterator_type::value_type;

    static compact_iterator_type begin_compact(const polygon_type &t) { return t.begin_compact(); }

    static compact_iterator_type end_compact(const polygon_type &t) { return t.end_compact(); }

    static iterator_type begin_points(const polygon_type &t) { return t.begin(); }

    static iterator_type end_points(const polygon_type &t) { return t.end(); }

    static std::size_t size(const polygon_type &t) { return t.size(); }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static void set_points(polygon_type &t, iT start, iT stop, std::size_t n = 4) {
        t.set_points(start, stop, n);
    }

    template <typename iT,
              std::enable_if_t<std::is_same_v<coordinate_type, typename iT::value_type>, int> = 0>
    static void set_compact(polygon_type &t, iT start, iT stop, std::size_t n = 4) {
        t.set_compact(start, stop, n);
    }

    static void set_compact(polygon_type &t, std::vector<coordinate_type> &&data) {
        t = std::move(data);
    }

    template <typename iT,
              std::enable_if_t<std::is_same_v<coordinate_type, typename iT::value_type>, int> = 0>
    static polygon_type construct_compact(iT start, iT stop, std::size_t n = 4) {
        return {start, stop, n};
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static polygon_type construct(iT start, iT stop, std::size_t n = 4) {
        auto ans = polygon_type();
        ans.set_points(start, stop, n);
        return ans;
    }
};

} // namespace polygon
} // namespace cbag

#endif

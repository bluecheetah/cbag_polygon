// Copyright 2019 Blue Cheetah Analog Design Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef CBAG_POLYGON_TRANSFORMATION_H
#define CBAG_POLYGON_TRANSFORMATION_H

#include <array>

#include <cbag/polygon/enum.h>
#include <cbag/polygon/point_data.h>

namespace cbag {
namespace polygon {

/** A class that handles rotations, reflections, and translations

    rotate/reflect first, then translates.
 */
template <typename T> class transformation {
  public:
    using coordinate_type = T;

  private:
    orientation orient_ = orientation::R0;
    point_data<coordinate_type> p_ = {0, 0};

  public:
    explicit transformation(coordinate_type x = 0, coordinate_type y = 0,
                            orientation orient = orientation::R0) noexcept
        : orient_{orient}, p_{x, y} {}

    explicit transformation(point_data<coordinate_type> p,
                            orientation orient = orientation::R0) noexcept
        : orient_{orient}, p_{std::move(p)} {}

    orientation orient() const noexcept { return orient_; }

    point_data<coordinate_type> offset() const noexcept { return p_; }

    coordinate_type coord(orientation_2d orient) const noexcept { return p_[orient]; }

    point_data<coordinate_type> transform(coordinate_type x, coordinate_type y) const noexcept {
        auto ans = _rotate_point(x, y);
        return ans += p_;
    }

    transformation &operator+=(const transformation &rhs) noexcept {
        p_ = rhs.transform(p_[0], p_[1]);
        orient_ = orient_ + rhs.orient_;
        return *this;
    }

    transformation &invert() noexcept {
        orient_ = -orient_;
        p_ = _rotate_point(-p_[0], -p_[1]);
        return *this;
    }

    transformation &move_by(coordinate_type dx, coordinate_type dy) noexcept {
        cbag::polygon::move_by(p_, dx, dy);
        return *this;
    }

  private:
    point_data<coordinate_type> _rotate_point(coordinate_type x, coordinate_type y) const noexcept {
        // decode orient
        auto code = static_cast<int>(orient_);
        auto swap = code >> 2;
        auto mx = code & 1;
        auto my = (code >> 1) & 1;

        // swap axes, then mirror
        auto new_x = swap * y + (swap ^ 1) * x;
        auto new_y = swap * x + (swap ^ 1) * y;
        new_x *= (1 - 2 * mx);
        new_y *= (1 - 2 * my);
        return {new_x, new_y};
    }
};

template <typename T> T x(const transformation<T> &obj) noexcept {
    return obj.coord(orientation_2d::X);
}

template <typename T> T y(const transformation<T> &obj) noexcept {
    return obj.coord(orientation_2d::Y);
}

template <typename T>
bool operator==(const transformation<T> &lhs, const transformation<T> &rhs) noexcept {
    return x(lhs) == x(rhs) && y(lhs) == y(rhs) && lhs.orient() == rhs.orient();
}

template <typename T>
bool operator!=(const transformation<T> &lhs, const transformation<T> &rhs) noexcept {
    return !(lhs == rhs);
}

template <typename T> transformation<T> operator-(const transformation<T> &obj) noexcept {
    auto ans = obj;
    return ans.invert();
}

template <typename T>
transformation<T> operator+(const transformation<T> &lhs, const transformation<T> &rhs) noexcept {
    auto ans = lhs;
    return ans += rhs;
}

template <typename T> bool is_null(const transformation<T> &obj) noexcept {
    return x(obj) == 0 && y(obj) == 0 && obj.orient() == orientation::R0;
}

template <typename T> std::string to_string(const transformation<T> &obj) {
    std::string ans = "Transform(";
    ans += std::to_string(x(obj));
    ans += ", ";
    ans += std::to_string(y(obj));
    ans += ", ";
    ans += cbag::to_string(obj.orient());
    ans += ")";
    return ans;
}

template <typename T> std::ostream &operator<<(std::ostream &stream, const transformation<T> &obj) {
    stream << to_string(obj);
    return stream;
}

template <typename T>
transformation<T> get_move_by(const transformation<T> &xform, T dx, T dy) noexcept {
    auto ans = xform;
    return ans.move_by(dx, dy);
}

template <typename T> transformation<T> get_inverse(const transformation<T> &xform) noexcept {
    auto ans = xform;
    return ans.invert();
}

template <typename T>
transformation<T> &operator+=(transformation<T> &lhs, orientation orient) noexcept {
    return (lhs += transformation(0, 0, orient));
}

} // namespace polygon
} // namespace cbag

#endif

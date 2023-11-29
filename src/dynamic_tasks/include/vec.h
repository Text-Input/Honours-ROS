#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>

struct Vec {
    double x, y, z;

    Vec() : Vec(0, 0, 0) {}

    Vec(double x, double y, double z) : x(x), y(y), z(z) {}

    explicit Vec(geometry_msgs::msg::Point point) : x(point.x), y(point.y), z(point.z) {}
    explicit Vec(geometry_msgs::msg::Vector3 point) : x(point.x), y(point.y), z(point.z) {}

    Vec operator-(const Vec &rhs) const {
        Vec vec{this->x - rhs.x, this->y - rhs.y, this->z - rhs.z};

        return vec;
    }

    Vec &operator/=(const double &val) {
        x /= val;
        y /= val;
        z /= val;

        return *this;
    }

    Vec operator*(const double &val) const {
        Vec vec{x * val, y * val, z * val};

        return vec;
    }

    Vec operator+(const Vec &val) const {
        Vec vec{x + val.x, y + val.y, z + val.z};

        return vec;
    }

    Vec &operator*=(const double &val) {
        x *= val;
        y *= val;
        z *= val;

        return *this;
    }

    bool operator==(const Vec&) const = default;

    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
};
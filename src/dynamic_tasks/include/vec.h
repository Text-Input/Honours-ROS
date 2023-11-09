#pragma once

#include <cmath>

struct Vec {
    double x, y, z;

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

    Vec &operator*=(const double &val) {
        x *= val;
        y *= val;
        z *= val;

        return *this;
    }

    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
};
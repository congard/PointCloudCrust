#ifndef POINTCLOUDCRUST_VECTOR_H
#define POINTCLOUDCRUST_VECTOR_H

#include <cmath>

#include "types.h"

namespace congard::PointCloudCrust {
class Vector {
public:
    inline Vector() = default;
    inline explicit Vector(float_n v): x(v), y(v), z(v) {}
    inline Vector(float_n x, float_n y, float_n z): x(x), y(y), z(z) {}

    inline Vector operator+(const Vector &rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    inline Vector operator-(const Vector &rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    inline Vector operator*(float_n multiplier) const {
        return {x * multiplier, y * multiplier, z * multiplier};
    }

    inline Vector operator/(float_n divider) const {
        return {x / divider, y / divider, z / divider};
    }

    inline Vector normalize() const {
        float_n factor = 1 / length();

        return {
            x * factor,
            y * factor,
            z * factor
        };
    }

    inline float_n length() const {
        return std::sqrt(length2());
    }

    inline float_n length2() const {
        return x * x + y * y + z * z;
    }

    inline bool isZero() const {
        return x == float_n(0) && y == float_n(0) && z == float_n(0);
    }

    inline static Vector cross(const Vector &lhs, const Vector &rhs) {
        return {
            lhs.y * rhs.z - lhs.z * rhs.y,
            rhs.x * lhs.z - rhs.z * lhs.x,
            lhs.x * rhs.y - lhs.y * rhs.x
        };
    }

    inline static float_n dot(const Vector &v1, const Vector &v2) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

public:
    float_n x;
    float_n y;
    float_n z;
};
}

#endif //POINTCLOUDCRUST_VECTOR_H

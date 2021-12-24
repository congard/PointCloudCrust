#ifndef POINTCLOUDCRUST_POINTS_H
#define POINTCLOUDCRUST_POINTS_H

#include <algorithm>
#include <cstring>

#include "Vector.h"

namespace congard::PointCloudCrust {
class Points {
public:
    inline Points() = default;
    Points(float_n *coords, size_t size, bool copy = true, bool takeOwnership = true);
    Points(const Points&) = delete;
    Points(Points &&rhs) noexcept;
    ~Points();

    Points& operator=(const Points&) = delete;
    Points& operator=(Points &&rhs) noexcept;
    inline Vector& operator[](size_t index) { return getPoint(index); }
    inline const Vector& operator[](size_t index) const { return getPoint(index); }

    inline size_t size() const { return m_size / 3; }

    inline float_n* getCoords() const { return m_coords; }

    Vector& getPoint(size_t index);
    const Vector& getPoint(size_t index) const;

private:
    float_n *m_coords {};
    size_t m_size {};
    bool m_ownership {false};
};

inline Points::Points(float_n *coords, size_t size, bool copy, bool takeOwnership) {
    m_size = size;
    m_ownership = takeOwnership;

    if (copy) {
        m_coords = new float_n[size];
        memcpy(m_coords, coords, size * sizeof(float_n));
    } else {
        m_coords = coords;
    }
}

inline Points::Points(Points &&rhs) noexcept {
    operator=(std::forward<Points&&>(rhs));
}

inline Points::~Points() {
    if (m_ownership) {
        delete[] m_coords;
    }
}

inline Points& Points::operator=(Points &&rhs) noexcept {
    std::swap(m_coords, rhs.m_coords);
    std::swap(m_size, rhs.m_size);
    std::swap(m_ownership, rhs.m_ownership);
    return *this;
}

inline Vector& Points::getPoint(size_t index) {
    return reinterpret_cast<Vector&>(m_coords[index * 3]);
}

inline const Vector& Points::getPoint(size_t index) const {
    return reinterpret_cast<const Vector&>(m_coords[index * 3]);
}
}

#endif //POINTCLOUDCRUST_POINTS_H

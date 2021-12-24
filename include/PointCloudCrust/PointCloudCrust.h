#ifndef POINTCLOUDCRUST_POINTCLOUDCRUST_H
#define POINTCLOUDCRUST_POINTCLOUDCRUST_H

#include <forward_list>
#include <array>
#include <mutex>

#include "Points.h"

namespace congard::PointCloudCrust {
class PointCloudCrust {
public:
    struct Triangle {
        int v1;
        int v2;
        int v3;
    };

    using Triangles = std::forward_list<Triangle>;

public:
    inline void setRadius(float_n radius) { m_radius = radius; }
    inline float_n getRadius() const { return m_radius; }

    void setPoints(float_n *coords, size_t size, bool copy = true, bool takeOwnership = true);
    inline void setPoints(Points &points) { std::swap(points, m_points); }
    inline const Points& getPoints() const { return m_points; }

    inline const Triangles& getTriangles() const { return m_triangles; }

    inline void reset() { m_triangles = {}; }

    void compute();
    void computeRange(float_n begin, float_n end);

private:
    using limits = std::numeric_limits<float_n>;

    bool analyzeTriangle(const Vector &a, const Vector &b, const Vector &c);
    std::array<Vector, 2> ballCenter(const Vector &a, const Vector &b, const Vector &c, const Vector &m) const;
    static Vector triangleCenter(const Vector &a, const Vector &b, const Vector &c);

private:
    Points m_points;
    Triangles m_triangles;
    float_n m_radius {};
    std::mutex m_trianglesMutex;
};

inline void PointCloudCrust::setPoints(float_n *coords, size_t size, bool copy, bool takeOwnership) {
    m_points = Points(coords, size, copy, takeOwnership);
}
}

#endif //POINTCLOUDCRUST_POINTCLOUDCRUST_H

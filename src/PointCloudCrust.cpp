#include "PointCloudCrust/PointCloudCrust.h"

namespace congard::PointCloudCrust {
void PointCloudCrust::compute() {
    auto n = m_points.size();

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            for (int k = j + 1; k < n; ++k) {
                if (analyzeTriangle(m_points[i], m_points[j], m_points[k])) {
                    m_triangles.emplace_front(Triangle {i, j, k});
                }
            }
        }
    }
}

// Q: https://stackoverflow.com/questions/70413446/linearize-nested-for-loops
// A: https://stackoverflow.com/a/70464293/9200394

void PointCloudCrust::computeRange(float_n begin, float_n end) {
    auto divide = [](float_n pos, size_t len) -> float_n {
        auto n = static_cast<float_n>(len);

        if (pos == 1) {
            return n;
        }

        if (pos == 0) {
            return 0;
        }

        // solve   x * (x - 1) * (x - 2) = n * (n - 1) * (n - 2) * pos   for x
        // https://en.wikipedia.org/wiki/Bisection_method

        float_n d = n * (n - 1) * (n - 2) * (1 - pos);

        auto f = [d](float_n x) {
            return std::pow(x, 3) - 3 * std::pow(x, 2) + 2 * x - d;
        };

        float_n a = 0;
        float_n b = n;
        float_n epsilon = 0.1;

        float_n x = 0;

        while (std::abs(a - b) > epsilon) {
            x = (a + b) / 2;

            if (std::abs(f(x)) <= epsilon) {
                break;
            } else if (f(x) * f(a) < 0) {
                b = x;
            } else {
                a = x;
            }
        }

        return std::ceil(n - x);
    };

    Triangles triangles;

    auto n = m_points.size();
    auto rangeBegin = static_cast<int>(divide(begin, n));
    auto rangeEnd = static_cast<int>(divide(end, n));

    for (int i = rangeBegin; i < rangeEnd; ++i) {
        for (int j = i + 1; j < n; ++j) {
            for (int k = j + 1; k < n; ++k) {
                if (analyzeTriangle(m_points[i], m_points[j], m_points[k])) {
                    triangles.emplace_front(Triangle {i, j, k});
                }
            }
        }
    }

    m_trianglesMutex.lock();
    m_triangles.splice_after(m_triangles.cbefore_begin(), triangles);
    m_trianglesMutex.unlock();
}

bool PointCloudCrust::analyzeTriangle(const Vector &a, const Vector &b, const Vector &c) {
    Vector m = triangleCenter(a, b, c);

    if (std::isnan(m.x))
        return false;

    auto ballCenters = ballCenter(a, b, c, m);

    if (std::isnan(ballCenters[0].x))
        return false;

    auto tolerance = float_n(0.01);

    // keep flag (result)
    bool keep = true;

    // analyze first ball
    {
        const Vector &ball = ballCenters[0];

        for (int i = 0; i < m_points.size() && keep; i++) {
            if ((ball - m_points[i]).length() < m_radius - tolerance) {
                keep = false;
            }
        }
    }

    // analyze second ball (if required)
    if (!keep) {
        // reset flag
        keep = true;

        const Vector &ball = ballCenters[1];

        for (int i = 0; i < m_points.size() && keep; i++) {
            if ((ball - m_points[i]).length() < m_radius - tolerance) {
                keep = false;
            }
        }
    }

    // return result
    return keep;
}

std::array<Vector, 2> PointCloudCrust::ballCenter(const Vector &a, const Vector &b, const Vector &c, const Vector &m) const {
    // vector: point A to triangle center M
    Vector am = m - a;

    // length: triangle center (m) to ball center (h) (pythagoras)
    auto amLen = am.length();

    if (amLen > m_radius) {
        // ball is too small
        return {
            Vector(limits::quiet_NaN()),
            Vector(limits::quiet_NaN())
        };
    }

    auto mhLen = std::sqrt(m_radius * m_radius - amLen * amLen);

    // compute orthogonal vector of triangle
    Vector ab = b - a;
    Vector ortho = Vector::cross(am, ab);

    if (ortho.isZero()) {
        Vector ac = c - a;
        ortho = Vector::cross(am, ac);
    }

    // normalized vector: triangle center M to ball center H
    Vector mhNorm = ortho.normalize();

    // compute and return centers of the two balls (vector addition)
    Vector mh_1 = mhNorm * mhLen;
    Vector mh_2 = mhNorm * -mhLen;

    return {
        m + mh_1,
        m + mh_2
    };
}

Vector PointCloudCrust::triangleCenter(const Vector &a, const Vector &b, const Vector &c) {
    // Algorithm:
    //
    //         |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)]
    // m = a + ---------------------------------------------------------.
    //                            2 | (b-a)x(c-a) |^2
    //
    // by Jonathan R Shewchuk
    // http://www.ics.uci.edu/~eppstein/junkyard/circumcenter.html

    Vector ac = c - a;
    Vector ab = b - a;

    Vector abXac = Vector::cross(ab, ac);

    if (abXac.isZero())
        return Vector(limits::quiet_NaN());

    // v1 = |c-a|^2 [(b-a)x(c-a)]x(b-a)
    Vector v1 = Vector::cross(abXac, ab) * ac.length2();

    // v2 = |b-a|^2 (c-a)x[(b-a)x(c-a)]
    Vector v2 = Vector::cross(ac, abXac) * ab.length2();

    // f = 2 | (b-a)x(c-a) |^2
    auto f = 2 * abXac.length2();

    // get final vector
    Vector v = (v1 + v2) / f;

    return a + v;
}
}

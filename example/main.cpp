#include <fstream>
#include <chrono>
#include <thread>
#include <vector>

#include <PointCloudCrust/PointCloudCrust.h>

using namespace congard::PointCloudCrust;

int main() {
    auto time = []() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    };

    std::vector<float_n> pointCloud;

    std::fstream fs("sphere.txt");

    float_n x, y, z;

    while (fs >> x >> y >> z) {
        pointCloud.emplace_back(x);
        pointCloud.emplace_back(y);
        pointCloud.emplace_back(z);
    }

    auto start = time();

    PointCloudCrust pointCloudCrust;
    pointCloudCrust.setRadius(0.3f);
    pointCloudCrust.setPoints(pointCloud.data(), pointCloud.size(), false, false);

    // multithreaded mode

    auto n = 8;
    std::thread workers[n];

    for (int i = 0; i < n; ++i) {
        workers[i] = std::thread([&](int i) {
            pointCloudCrust.computeRange((float) i / n, (float) (i + 1) / n);
        }, i);
    }

    for (auto &worker : workers) {
        worker.join();
    }

    // single-threaded mode
    // pointCloudCrust.compute();

    pointCloudCrust.optimize();

    printf("End, time: %li\n", time() - start);

    // print the result in OBJ file format

    auto &points = pointCloudCrust.getPoints();

    for (int i = 0; i < points.size(); ++i) {
        auto &point = points.getPoint(i);
        printf("v %f %f %f\n", point.x, point.y, point.z);
    }

    for (auto &t : pointCloudCrust.getTriangles()) {
        printf("f %i %i %i\n", t.v1 + 1, t.v2 + 1, t.v3 + 1);
    }

    return 0;
}

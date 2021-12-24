#ifndef POINTCLOUDCRUST_TYPES_H
#define POINTCLOUDCRUST_TYPES_H

namespace congard::PointCloudCrust {
#ifdef POINTCLOUDCRUST_DOUBLE_PRECISION
using float_n = double;
#else
using float_n = float;
#endif
}

#endif //POINTCLOUDCRUST_TYPES_H

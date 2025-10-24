#include <Eigen/Core>

namespace amp {
Eigen::Vector2d convert(const Eigen::Matrix<double, 2, 1>& v) {
    return Eigen::Vector2d(v[0], v[1]);
}
}

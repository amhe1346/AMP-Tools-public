#include "HelpfulClass.h"
#include "minowskisum.h"
#include <cmath>
#include <iostream>

namespace amp {

std::vector<Eigen::Vector2d> makeCirclePolygon(const Eigen::Vector2d& center, double radius, int num_points) {
    std::vector<Eigen::Vector2d> poly;
    for (int i = 0; i < num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;
        poly.emplace_back(center.x() + radius * std::cos(theta),
                         center.y() + radius * std::sin(theta));
    }
    return poly;
}

std::vector<Eigen::Vector2d> computeCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle,
                                                   const std::vector<Eigen::Vector2d>& robot) {
    // Convert Eigen::Vector2d to std::pair<double, double>
    std::vector<std::pair<double, double>> obs_poly, robot_poly;
    for (const auto& v : obstacle) obs_poly.emplace_back(v.x(), v.y());
    for (const auto& v : robot) robot_poly.emplace_back(v.x(), v.y());
    // Use classic Minkowski sum: reflect robot about origin
    std::vector<Eigen::Vector2d> robot_reflected;
    for (const auto& v : robot) {
        Eigen::Vector2d v_unit = v.normalized();
        double v_len = v.norm() + 0.1;
        robot_reflected.push_back(-v_unit * v_len);
    }
    auto result_poly = minkowskiSumEigen(obstacle, robot_reflected);
    return result_poly;
}

} // namespace amp
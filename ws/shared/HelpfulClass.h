#pragma once
#include <vector>
#include <Eigen/Dense>

namespace amp {

// Returns a polygonal approximation of a circle centered at 'center' with given 'radius' and 'num_points'.
std::vector<Eigen::Vector2d> makeCirclePolygon(const Eigen::Vector2d& center, double radius, int num_points);

// Computes the C-space obstacle as the Minkowski sum of the obstacle and robot polygons.
// Both polygons should be in counterclockwise order.
std::vector<Eigen::Vector2d> computeCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle,
                                                   const std::vector<Eigen::Vector2d>& robot);

} // namespace amp
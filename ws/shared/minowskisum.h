
#ifndef MINOWSKISUM_H
#define MINOWSKISUM_H

#include <vector>
#include <utility>
#include <Eigen/Dense>

// Minkowski sum utility functions for C-space obstacle expansion
// Add your function declarations and documentation here


// Expand (buffer) a convex polygon by a given distance (outward offset)
// Returns the buffered convex polygon in counterclockwise order.
std::vector<std::pair<double, double>> bufferConvexPolygon(
    const std::vector<std::pair<double, double>>& polygon, double buffer);

// Compute the Minkowski sum of two convex polygons (robot centered at origin)
// Both polygons must be ordered counterclockwise and convex.
// Returns the resulting convex polygon in counterclockwise order.
std::vector<std::pair<double, double>> convexMinkowskiSum(
    const std::vector<std::pair<double, double>>& robot,
    const std::vector<std::pair<double, double>>& obstacle);

// Compute the Minkowski sum with a buffer added to the obstacle
std::vector<std::pair<double, double>> convexMinkowskiSumWithBuffer(
    const std::vector<std::pair<double, double>>& robot,
    const std::vector<std::pair<double, double>>& obstacle,
    double buffer);

// (Legacy) Compute the Minkowski sum of two polygons (all pairwise sums)
std::vector<std::pair<double, double>> minkowskiSum(
    const std::vector<std::pair<double, double>>& poly1,
    const std::vector<std::pair<double, double>>& poly2);

// Minkowski sum for Eigen::Vector2d polygons (classic: both polygons as arguments)
std::vector<Eigen::Vector2d> minkowskiSumEigen(
    const std::vector<Eigen::Vector2d>& obstacle,
    const std::vector<Eigen::Vector2d>& robot);

#endif // MINOWSKISUM_H

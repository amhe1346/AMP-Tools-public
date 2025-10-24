#include "tools/Obstacle.h"

namespace amp {
Polygon::Polygon(std::vector<Eigen::Vector2d>& vertices_ccw) {
    m_vertices_ccw = vertices_ccw;
}

std::vector<Eigen::Vector2d>& Polygon::verticesCCW() {
    return m_vertices_ccw;
}

const std::vector<Eigen::Vector2d>& Polygon::verticesCCW() const {
    return m_vertices_ccw;
}
}

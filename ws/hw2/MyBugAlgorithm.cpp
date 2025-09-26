




#include "hw/HW2.h"
#include "MyBugAlgorithm.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

#include "AMPCore.h"
#include <optional>


// QHits struct and global q for m_line and circumnavigate_obstacle
struct QHits {
    std::vector<Eigen::Vector2d> hits;
};
QHits q;

void m_line(const amp::Problem2D& problem, amp::Path2D& path);
bool collision_check(const amp::Problem2D& problem, const Eigen::Vector2d& pt);
Eigen::Vector2d offset_obstacle(const amp::Problem2D& problem, const Eigen::Vector2d& obstacle_pt);
bool segments_intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
void retrace_steps(const std::vector<Eigen::Vector2d>& circumnav_waypoints, const Eigen::Vector2d& mline_point, amp::Path2D& path, const amp::Problem2D& problem);
void m_line_intersections(const amp::Problem2D& problem);

// Helper: returns true if segments (p1,p2) and (q1,q2) intersect
bool segments_intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return a[0]*b[1] - a[1]*b[0]; };
    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;
    double denom = cross(r, s);
    if (std::abs(denom) < 1e-10) return false; // parallel or collinear
    double t = cross(q1 - p1, s) / denom;
    double u = cross(q1 - p1, r) / denom;
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}
// Forward declaration
// defins if its outside of the workspace boundaries
bool is_out_of_bounds(const amp::Problem2D& problem, const Eigen::Vector2d& pt);
// Returns true if pt is outside the workspace boundaries
bool is_out_of_bounds(const amp::Problem2D& problem, const Eigen::Vector2d& pt) {
    double x = pt[0];
    double y = pt[1];
    double min_x = problem.x_min;
    double max_x = problem.x_max;
    double min_y = problem.y_min;
    double max_y = problem.y_max;
    return (x < min_x || x > max_x || y < min_y || y > max_y);
}
static bool point_in_polygon(const Eigen::Vector2d& pt, const std::vector<Eigen::Vector2d>& verts);
// Helper: returns true if pt is inside the polygon defined by verts
static bool point_in_polygon(const Eigen::Vector2d& pt, const std::vector<Eigen::Vector2d>& verts) {
    int crossings = 0;
    size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector2d& v1 = verts[i];
        const Eigen::Vector2d& v2 = verts[(i + 1) % n];
        if (((v1.y() > pt.y()) != (v2.y() > pt.y())) &&
            (pt.x() < (v2.x() - v1.x()) * (pt.y() - v1.y()) / (v2.y() - v1.y() + 1e-12) + v1.x())) {
            crossings++;
        }
    }
    return (crossings % 2) == 1;
}

// Plots m-line, stops at obstacle, offsets, and circumnavigates until it returns near a previous point
void circumnavigate_obstacle(const amp::Problem2D& problem, amp::Path2D& path) {
    // Subfunction: finds waypoints within 0.2 units of q.hits and returns the one closest to q_goal
    auto waypoint_to_mline = [&](const std::vector<Eigen::Vector2d>& waypoints, const std::vector<Eigen::Vector2d>& /*q_hits*/, const Eigen::Vector2d& q_goal) -> std::optional<Eigen::Vector2d> {
        if (waypoints.empty()) {
            return std::nullopt;
        }
        size_t min_idx = 0;
        double min_dist = (waypoints[0] - q_goal).norm();
        for (size_t i = 1; i < waypoints.size(); ++i) {
            double dist = (waypoints[i] - q_goal).norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }
    // ...existing code...
        return waypoints[min_idx];
    };
    std::vector<Eigen::Vector2d>& waypoints = path.waypoints;
    Eigen::Vector2d start = problem.q_init;
    Eigen::Vector2d goal = problem.q_goal;
    double distance = (goal - start).norm();
    Eigen::Vector2d direction = goal - start;
    if (distance == 0) {
        // Only add start if not inside any obstacle
        bool start_inside_any = false;
        for (const auto& obstacle : problem.obstacles) {
            if (point_in_polygon(start, obstacle.verticesCW())) {
                start_inside_any = true;
                // ...debug removed...
                break;
            }
        }
        if (!start_inside_any) {
            waypoints.push_back(start);
            // ...debug removed...
        }
    // ...debug removed...
    return;
    }
    direction.normalize();
    double step = 0.2;
    for (double d = 0.0; d < distance; d += step) {
        Eigen::Vector2d pt = start + d * direction;
        if (collision_check(problem, pt)) {
            // Obstacle encountered, offset and begin circumnavigation
            Eigen::Vector2d circ_pt = offset_obstacle(problem, pt);
            // Only add circ_pt if not inside any obstacle
            bool circ_inside_any = false;
            for (const auto& obstacle : problem.obstacles) {
                if (point_in_polygon(circ_pt, obstacle.verticesCW())) {
                    circ_inside_any = true;
                    // ...debug removed...
                    break;
                }
            }
            if (!circ_inside_any) {
                waypoints.push_back(circ_pt);
                // ...debug removed...
            }
            // Circumnavigate until we return near this point or back at start
            Eigen::Vector2d circ_start = circ_pt;
            int circ_count = 0;
            Eigen::Vector2d last_dir = {0, 0.2}; // Start preferring 'up'
            std::vector<Eigen::Vector2d> circumnav_waypoints;
            circumnav_waypoints.push_back(circ_pt);
            bool ended_by_self_intersection = false;
            while (true) {
                std::vector<Eigen::Vector2d> circ_dirs_all = {
                    // 8 directions (cardinal and diagonal), radius 0.2
                    {0.2, 0}, {0.1414, 0.1414}, {0, 0.2}, {-0.1414, 0.1414},
                    {-0.2, 0}, {-0.1414, -0.1414}, {0, -0.2}, {0.1414, -0.1414}
                };
                // Prevent moving directly back (180-degree reversal from last_dir)
                std::vector<Eigen::Vector2d> circ_dirs;
                for (const auto& dir : circ_dirs_all) {
                    // If last_dir is zero (first step), allow all directions
                    if (last_dir.norm() < 1e-8) {
                        circ_dirs.push_back(dir);
                        continue;
                    }
                    // Compute cosine of angle between dir and last_dir
                    double cos_angle = dir.dot(last_dir) / (dir.norm() * last_dir.norm() + 1e-12);
                    // Exclude directions that are more than ~135 degrees apart (cos(angle) < -0.707)
                    if (cos_angle > -0.707) {
                        circ_dirs.push_back(dir);
                    }
                }
                // 1. Prefer continuing in the same direction
                std::vector<Eigen::Vector2d> try_dirs;
                if (std::find(circ_dirs.begin(), circ_dirs.end(), last_dir) != circ_dirs.end()) {
                    try_dirs.push_back(last_dir);
                }
                // 2. Prefer clockwise (right turn) order for remaining directions
                // Start at 'up' (0, 0.2) and proceed clockwise
                int up_idx = -1;
                for (size_t i = 0; i < circ_dirs.size(); ++i) {
                    if ((circ_dirs[i] - Eigen::Vector2d(0, 0.2)).norm() < 1e-6) {
                        up_idx = static_cast<int>(i);
                        break;
                    }
                }
                if (up_idx != -1) {
                    for (size_t j = 0; j < circ_dirs.size(); ++j) {
                        size_t idx = (up_idx + j) % circ_dirs.size();
                        const auto& dir = circ_dirs[idx];
                        if (std::find(try_dirs.begin(), try_dirs.end(), dir) == try_dirs.end()) {
                            try_dirs.push_back(dir);
                        }
                    }
                } else {
                    // Fallback: just append in order if 'up' not found
                    for (const auto& dir : circ_dirs) {
                        if (std::find(try_dirs.begin(), try_dirs.end(), dir) == try_dirs.end()) {
                            try_dirs.push_back(dir);
                        }
                    }
                }
                // ...existing code...
                bool moved = false;
                for (const auto& dir : try_dirs) {
                    Eigen::Vector2d candidate = circ_pt + dir;
                    // Only plot if within 0.2 units of an obstacle edge (but not inside)
                    bool near_edge = false;
                    for (const auto& obstacle : problem.obstacles) {
                        const auto& verts = obstacle.verticesCW();
                        if (point_in_polygon(candidate, verts)) continue; // skip if inside
                        for (size_t i = 0; i < verts.size(); ++i) {
                            const Eigen::Vector2d& v1 = verts[i];
                            const Eigen::Vector2d& v2 = verts[(i + 1) % verts.size()];
                            Eigen::Vector2d edge = v2 - v1;
                            Eigen::Vector2d to_pt = candidate - v1;
                            double t = edge.dot(to_pt) / (edge.squaredNorm() + 1e-12);
                            t = std::max(0.0, std::min(1.0, t));
                            Eigen::Vector2d proj = v1 + t * edge;
                            double dist = (candidate - proj).norm();
                            if (dist <= 0.2) {
                                near_edge = true;
                                break;
                            }
                        }
                        if (near_edge) break;
                    }
                    // ...existing code...
                    if (near_edge) {
                        // Check if candidate is out of bounds
                        if (is_out_of_bounds(problem, candidate)) {
                            // ...debug removed...
                            // Attempt to retrace steps to the closest m-line point
                            auto mline_point_opt = waypoint_to_mline(circumnav_waypoints, q.hits, goal);
                            if (mline_point_opt) {
                                retrace_steps(circumnav_waypoints, *mline_point_opt, path, problem);
                            }
                            return;
                        }
                        // Check if candidate is inside any obstacle before adding
                        bool inside_any = false;
                        for (const auto& obstacle : problem.obstacles) {
                            if (point_in_polygon(candidate, obstacle.verticesCW())) {
                                inside_any = true;
                                // ...debug removed...
                                break;
                            }
                        }
                        if (!inside_any) {
                            circ_pt = candidate;
                            waypoints.push_back(circ_pt);
                            circumnav_waypoints.push_back(circ_pt);
                            last_dir = dir;
                            moved = true;
                            // ...debug removed...
                            break;
                        }
                        // If inside, skip this candidate and try next direction
                    }
                }
                circ_count++;
                if (!moved) {
                    // Check if the current point is too far from any obstacle edge
                    bool near_any_edge = false;
                    for (const auto& obstacle : problem.obstacles) {
                        const auto& verts = obstacle.verticesCW();
                        for (size_t i = 0; i < verts.size(); ++i) {
                            const Eigen::Vector2d& v1 = verts[i];
                            const Eigen::Vector2d& v2 = verts[(i + 1) % verts.size()];
                            Eigen::Vector2d edge = v2 - v1;
                            Eigen::Vector2d to_pt = circ_pt - v1;
                            double t = edge.dot(to_pt) / (edge.squaredNorm() + 1e-12);
                            t = std::max(0.0, std::min(1.0, t));
                            Eigen::Vector2d proj = v1 + t * edge;
                            double dist = (circ_pt - proj).norm();
                            if (dist <= 0.2) {
                                near_any_edge = true;
                                break;
                            }
                        }
                        if (near_any_edge) break;
                    }
                    if (!near_any_edge) {
                        // Restart circumnavigate_obstacle from current point
                        amp::Problem2D restart_problem = problem;
                        restart_problem.q_init = circ_pt;
                        circumnavigate_obstacle(restart_problem, path);
                        return;
                    }
                    // ...debug removed...
                    // Retrace to the first circumnavigation point (circ_pt)
                    if (!circumnav_waypoints.empty()) {
                        waypoints.push_back(circumnav_waypoints.front());
                    }
                    // After retracing, attempt CCW behavior
                    std::vector<Eigen::Vector2d> ccw_dirs = try_dirs;
                    std::reverse(ccw_dirs.begin(), ccw_dirs.end());
                    bool moved_ccw = false;
                    for (const auto& dir : ccw_dirs) {
                        Eigen::Vector2d candidate = circ_pt + dir;
                        bool near_edge = false;
                        for (const auto& obstacle : problem.obstacles) {
                            const auto& verts = obstacle.verticesCW();
                            if (point_in_polygon(candidate, verts)) continue;
                            for (size_t j = 0; j < verts.size(); ++j) {
                                const Eigen::Vector2d& v1 = verts[j];
                                const Eigen::Vector2d& v2 = verts[(j + 1) % verts.size()];
                                Eigen::Vector2d edge = v2 - v1;
                                Eigen::Vector2d to_pt = candidate - v1;
                                double t = edge.dot(to_pt) / (edge.squaredNorm() + 1e-12);
                                t = std::max(0.0, std::min(1.0, t));
                                Eigen::Vector2d proj = v1 + t * edge;
                                double dist = (candidate - proj).norm();
                                if (dist <= 0.2) {
                                    near_edge = true;
                                    break;
                                }
                            }
                            if (near_edge) break;
                        }
                        if (near_edge && !is_out_of_bounds(problem, candidate)) {
                            circ_pt = candidate;
                            waypoints.push_back(circ_pt);
                            circumnav_waypoints.push_back(circ_pt);
                            last_dir = dir;
                            moved_ccw = true;
                            break;
                        }
                    }
                    if (moved_ccw) {
                        circ_count++;
                        continue;
                    }
                    // If still stuck, attempt to retrace to m-line point as fallback
                    auto mline_point_opt = waypoint_to_mline(circumnav_waypoints, q.hits, goal);
                    if (mline_point_opt) {
                        retrace_steps(circumnav_waypoints, *mline_point_opt, path, problem);
                    }
                    return;
                }
                if (circ_count > 1000) {
                    // ...debug removed...
                    // ...debug removed...
                    return;
                }
                if ((circ_pt - circ_start).norm() < 1e-6 && circ_count > 1) {
                    // ...debug removed...
                    // ...debug removed...
                    return;
                }
                // Robust self-intersection check: skip for the first 5 circumnavigation waypoints
                if (waypoints.size() >= 4 && waypoints.size() > 5) {
                    const Eigen::Vector2d& seg_start = waypoints[waypoints.size() - 2];
                    const Eigen::Vector2d& seg_end = waypoints.back();
                    for (size_t i = 0; i + 2 < waypoints.size() - 1; ++i) {
                        // Check segment (waypoints[i], waypoints[i+1])
                        if (segments_intersect(seg_start, seg_end, waypoints[i], waypoints[i+1])) {
                            // ...debug removed...
                            if (circ_count == 4) {
                                // Retrace to the first circumnavigation point (circ_pt)
                                if (!circumnav_waypoints.empty()) {
                                    waypoints.push_back(circumnav_waypoints.front());
                                }
                                // After retracing, attempt CCW behavior
                                std::vector<Eigen::Vector2d> ccw_dirs = try_dirs;
                                std::reverse(ccw_dirs.begin(), ccw_dirs.end());
                                bool moved_ccw = false;
                                for (const auto& dir : ccw_dirs) {
                                    Eigen::Vector2d candidate = circ_pt + dir;
                                    bool near_edge = false;
                                    for (const auto& obstacle : problem.obstacles) {
                                        const auto& verts = obstacle.verticesCW();
                                        if (point_in_polygon(candidate, verts)) continue;
                                        for (size_t j = 0; j < verts.size(); ++j) {
                                            const Eigen::Vector2d& v1 = verts[j];
                                            const Eigen::Vector2d& v2 = verts[(j + 1) % verts.size()];
                                            Eigen::Vector2d edge = v2 - v1;
                                            Eigen::Vector2d to_pt = candidate - v1;
                                            double t = edge.dot(to_pt) / (edge.squaredNorm() + 1e-12);
                                            t = std::max(0.0, std::min(1.0, t));
                                            Eigen::Vector2d proj = v1 + t * edge;
                                            double dist = (candidate - proj).norm();
                                            if (dist <= 0.2) {
                                                near_edge = true;
                                                break;
                                            }
                                        }
                                        if (near_edge) break;
                                    }
                                    if (near_edge && !is_out_of_bounds(problem, candidate)) {
                                        circ_pt = candidate;
                                        waypoints.push_back(circ_pt);
                                        circumnav_waypoints.push_back(circ_pt);
                                        last_dir = dir;
                                        moved_ccw = true;
                                        break;
                                    }
                                }
                                if (moved_ccw) {
                                    circ_count++;
                                    continue;
                                }
                            }
                            ended_by_self_intersection = true;
                            goto after_circumnavigation;
                        }
                    }
                }
            }
            after_circumnavigation:
            if (ended_by_self_intersection) {
                auto mline_point_opt = waypoint_to_mline(circumnav_waypoints, q.hits, goal);
                if (mline_point_opt) {
                    retrace_steps(circumnav_waypoints, *mline_point_opt, path, problem);
                }
            }
            return;
        }
        // Only add pt if not inside any obstacle
        bool inside_any = false;
        for (const auto& obstacle : problem.obstacles) {
            if (point_in_polygon(pt, obstacle.verticesCW())) {
                inside_any = true;
                // ...debug removed...
                break;
            }
        }
        if (!inside_any) {
            waypoints.push_back(pt);
            // ...debug removed...
        }
    }
    // ...existing code...
    // Only add goal if not inside any obstacle
    bool goal_inside_any = false;
    for (const auto& obstacle : problem.obstacles) {
        if (point_in_polygon(goal, obstacle.verticesCW())) {
            goal_inside_any = true;
        // ...debug removed...
            break;
        }
    }
    if (!goal_inside_any) {
        waypoints.push_back(goal);
        // ...debug removed...
    }
}


// Plots a straight line from q_init to q_goal, adding waypoints every 0.2 units
// Refined m_line: stores all intersection points of m-line and obstacles in q.hits
void m_line(const amp::Problem2D& problem, amp::Path2D& path) {
    Eigen::Vector2d start = problem.q_init;
    Eigen::Vector2d goal = problem.q_goal;
    double distance = (goal - start).norm();
    Eigen::Vector2d direction = goal - start;
    if (distance == 0) {
        path.waypoints.push_back(start);
        return;
    }
    direction.normalize();
    double step = 0.2;
    for (double d = 0.0; d < distance; d += step) {
        Eigen::Vector2d pt = start + d * direction;
        bool inside_any = false;
        for (const auto& obstacle : problem.obstacles) {
            if (point_in_polygon(pt, obstacle.verticesCW())) {
                inside_any = true;
                // ...debug removed...
                break;
            }
        }
        if (!inside_any) {
            // ...debug removed...
            path.waypoints.push_back(pt);
        }
    }
    bool goal_inside_any = false;
    for (const auto& obstacle : problem.obstacles) {
        if (point_in_polygon(goal, obstacle.verticesCW())) {
            goal_inside_any = true;
            // ...debug removed...
            break;
        }
    }
    if (!goal_inside_any) {
    // ...debug removed...
        path.waypoints.push_back(goal);
    }
}
// Returns true if any obstacle is within 0.2 units of the given point (x, y)
bool collision_check(const amp::Problem2D& problem, const Eigen::Vector2d& pt) {
    // Only check obstacles that are between pt and q_goal
    const Eigen::Vector2d& goal = problem.q_goal;
    double min_x = std::min(pt[0], goal[0]);
    double max_x = std::max(pt[0], goal[0]);
    double min_y = std::min(pt[1], goal[1]);
    double max_y = std::max(pt[1], goal[1]);
    int obs_idx = 0;
    // Retrieve the previous point if available (for segment crossing check)
    static Eigen::Vector2d prev_pt = pt; // fallback if not set
    static bool prev_set = false;
    bool segment_crosses = false;
    for (const auto& obstacle : problem.obstacles) {
        const auto& verts = obstacle.verticesCW();
        // Compute obstacle bounding box
        double obs_min_x = verts[0][0], obs_max_x = verts[0][0];
        double obs_min_y = verts[0][1], obs_max_y = verts[0][1];
        for (const auto& v : verts) {
            obs_min_x = std::min(obs_min_x, v[0]);
            obs_max_x = std::max(obs_max_x, v[0]);
            obs_min_y = std::min(obs_min_y, v[1]);
            obs_max_y = std::max(obs_max_y, v[1]);
        }
        // Only check if obstacle bounding box overlaps segment bounding box
        if (obs_max_x < min_x || obs_min_x > max_x || obs_max_y < min_y || obs_min_y > max_y) {
            obs_idx++;
            continue;
        }
        // Point-in-polygon test
        if (point_in_polygon(pt, verts)) {
            // ...debug removed...
            return true;
        }
        // Edge proximity check
        for (size_t i = 0; i < verts.size(); ++i) {
            const Eigen::Vector2d& v1 = verts[i];
            const Eigen::Vector2d& v2 = verts[(i + 1) % verts.size()];
            // Check distance to edge segment
            Eigen::Vector2d edge = v2 - v1;
            Eigen::Vector2d to_pt = pt - v1;
            double t = edge.dot(to_pt) / (edge.squaredNorm() + 1e-12);
            t = std::max(0.0, std::min(1.0, t));
            Eigen::Vector2d proj = v1 + t * edge;
            double dist = (pt - proj).norm();
            if (dist <= 0.2) {
                // ...debug removed...
                return true;
            }
            // Segment crossing check: does (prev_pt, pt) cross (v1, v2)?
            if (prev_set) {
                auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return a[0]*b[1] - a[1]*b[0]; };
                Eigen::Vector2d r = pt - prev_pt;
                Eigen::Vector2d s = v2 - v1;
                double denom = cross(r, s);
                if (std::abs(denom) > 1e-10) {
                    double t_seg = cross(v1 - prev_pt, s) / denom;
                    double u_seg = cross(v1 - prev_pt, r) / denom;
                    if (t_seg >= 0 && t_seg <= 1 && u_seg >= 0 && u_seg <= 1) {
                        // ...debug removed...
                        segment_crosses = true;
                    }
                }
            }
        }
        obs_idx++;
    }
    prev_pt = pt;
    prev_set = true;
    if (segment_crosses) return true;
    return false;
}

// Returns a point offset by 0.2 units from the given obstacle point, not in collision
Eigen::Vector2d offset_obstacle(const amp::Problem2D& problem, const Eigen::Vector2d& obstacle_pt) {
    // Try up, right, left, down, and half-step diagonals between them
    std::vector<Eigen::Vector2d> directions = {
        {0, 0.2},        // up
        {0.1414, 0.1414},// up-right (0.2/sqrt(2))
        {0.2, 0},        // right
        {0.1414, -0.1414},// down-right
        {0, -0.2},       // down
        {-0.1414, -0.1414},// down-left
        {-0.2, 0},       // left
        {-0.1414, 0.1414} // up-left
    };
    for (const auto& dir : directions) {
        Eigen::Vector2d candidate = obstacle_pt + dir;
        if (!collision_check(problem, candidate)) {
            return candidate;
        }
    }
    // If all directions are blocked, return the original point (could not offset)
    return obstacle_pt;
}
// Retrace steps: plot waypoints in reverse order up to the point from waypoint_to_mline
void retrace_steps(const std::vector<Eigen::Vector2d>& circumnav_waypoints, const Eigen::Vector2d& mline_point, amp::Path2D& path, const amp::Problem2D& problem) {
    // Find the index of mline_point in circumnav_waypoints (within a small threshold)
    int idx = -1;
    for (int i = static_cast<int>(circumnav_waypoints.size()) - 1; i >= 0; --i) {
        if ((circumnav_waypoints[i] - mline_point).norm() < 1e-6) {
            idx = i;
            break;
        }
    }
    if (idx == -1) {
        return;
    }
    // Plot waypoints in reverse order from the end to idx (inclusive)
    for (int i = static_cast<int>(circumnav_waypoints.size()) - 1; i >= idx; --i) {
        bool inside_any = false;
        for (const auto& obstacle : problem.obstacles) {
            if (point_in_polygon(circumnav_waypoints[i], obstacle.verticesCW())) {
                inside_any = true;
                // ...debug removed...
                break;
            }
        }
        if (!inside_any) {
            path.waypoints.push_back(circumnav_waypoints[i]);
            // ...debug removed...
        }
    }
    // ...existing code...
}
// Calculates a straight line from q_init to q_goal, adds waypoints every 0.2 units, and stores all intersection points of m-line and obstacles in q.hits
void m_line_intersections(const amp::Problem2D& problem) {
    Eigen::Vector2d start = problem.q_init;
    Eigen::Vector2d goal = problem.q_goal;
    // Only compute and store intersection points in q.hits, do not plot anything
    q.hits.clear();
    for (const auto& obstacle : problem.obstacles) {
        const auto& verts = obstacle.verticesCW();
        for (size_t i = 0; i < verts.size(); ++i) {
            const Eigen::Vector2d& v1 = verts[i];
            const Eigen::Vector2d& v2 = verts[(i + 1) % verts.size()];
            // Check for intersection between m-line (start, goal) and edge (v1, v2)
            auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return a[0]*b[1] - a[1]*b[0]; };
            Eigen::Vector2d r = goal - start;
            Eigen::Vector2d s = v2 - v1;
            double denom = cross(r, s);
            if (std::abs(denom) < 1e-10) continue; // parallel or collinear
            double t = cross(v1 - start, s) / denom;
            double u = cross(v1 - start, r) / denom;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                Eigen::Vector2d intersection = start + t * r;
                q.hits.push_back(intersection);
            }
        }
    }
    // Optionally print hits for debug
    // ...existing code...
}


// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    m_line_intersections(problem);
    // Always try to add q_init if valid
    bool q_init_inside = false;
    for (const auto& obstacle : problem.obstacles) {
        if (point_in_polygon(problem.q_init, obstacle.verticesCW())) {
            q_init_inside = true;
            break;
        }
    }
    if (!q_init_inside) {
        if (path.waypoints.empty() || (path.waypoints.front() - problem.q_init).norm() > 1e-6) {
            path.waypoints.insert(path.waypoints.begin(), problem.q_init);
        }
    }
    // Loop circumnavigate_obstacle up to 10 times or until the goal is reached
    amp::Problem2D current_problem = problem;
    for (int i = 0; i < 10; ++i) {
        if (i > 0 && !path.waypoints.empty()) {
            current_problem.q_init = path.waypoints.back();
        }
        circumnavigate_obstacle(current_problem, path);
        if (!path.waypoints.empty() && (path.waypoints.back() - problem.q_goal).norm() < 1e-3) {
            break;
        }
    }
    // Always try to add q_goal if valid
    bool q_goal_inside = false;
    for (const auto& obstacle : problem.obstacles) {
        if (point_in_polygon(problem.q_goal, obstacle.verticesCW())) {
            q_goal_inside = true;
            break;
        }
    }
    if (!q_goal_inside) {
        if (path.waypoints.empty() || (path.waypoints.back() - problem.q_goal).norm() > 1e-6) {
            path.waypoints.push_back(problem.q_goal);
        }
    }
    return path;
}
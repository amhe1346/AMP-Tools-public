#include "../../include/AMPCore.h"
#include "../shared/MyTranslationalCSpace.h"
#include "../shared/MySamplingBasedPlanners.h"
#include <vector>
#include <iostream>

using namespace amp;

class DecentralPlanner {
public:
    // Each agent gets its own path
    std::vector<amp::Path2D> plan(const std::vector<Eigen::Vector2d>& q_inits,
                                  const std::vector<Eigen::Vector2d>& q_goals,
                                  double x_min, double x_max, double y_min, double y_max,
                                  const std::vector<amp::Polygon>& obstacles,
                                  const std::vector<double>& radii) {
        std::vector<amp::Path2D> agent_paths;
        std::vector<amp::Polygon> dynamic_obstacles = obstacles;
        int num_agents = q_inits.size();
        for (int i = 0; i < num_agents; ++i) {
            // Confirm agent-specific parameters
            const Eigen::Vector2d& agent_q_init = q_inits[i];
            const Eigen::Vector2d& agent_q_goal = q_goals[i];
            double agent_radius = radii[i];
            // Construct C-space for agent i with all current obstacles
            MyTranslationalCSpace cspace(
                80, 80, x_min, x_max, y_min, y_max, dynamic_obstacles, agent_radius
            );
            amp::Problem2D problem;
            problem.q_init = agent_q_init;
            problem.q_goal = agent_q_goal;
            problem.x_min = x_min;
            problem.x_max = x_max;
            problem.y_min = y_min;
            problem.y_max = y_max;
            problem.obstacles = dynamic_obstacles;
            // Use goal bias RRT
            MyRRT rrt;
            amp::Path2D path = rrt.plan(problem);
            // Ensure path starts at q_init and ends at q_goal
            if (path.waypoints.empty() || path.waypoints.front() != problem.q_init)
                path.waypoints.insert(path.waypoints.begin(), problem.q_init);
            if (path.waypoints.back() != problem.q_goal)
                path.waypoints.push_back(problem.q_goal);
            // Check if path is valid (no collision at start/end)
            bool valid_start = !amp::MyCollisionChecker::pointInObstacles(problem.q_init, dynamic_obstacles);
            bool valid_end = !amp::MyCollisionChecker::pointInObstacles(problem.q_goal, dynamic_obstacles);
            if (!valid_start || !valid_end) {
                // Minimal invalid path: just q_init and q_goal
                path.waypoints.clear();
                path.waypoints.push_back(problem.q_init);
                path.waypoints.push_back(problem.q_goal);
            }
            agent_paths.push_back(path);
            // Add swept volume: buffer each waypoint as a small circle obstacle BEFORE next agent plans
            for (const auto& pt : path.waypoints) {
                std::vector<Eigen::Vector2d> circle_vertices;
                int num_circle_pts = 12;
                for (int k = 0; k < num_circle_pts; ++k) {
                    double theta = 2.0 * M_PI * k / num_circle_pts;
                    circle_vertices.push_back(pt + agent_radius * Eigen::Vector2d(cos(theta), sin(theta)));
                }
                Polygon circle_obstacle(circle_vertices);
                dynamic_obstacles.push_back(circle_obstacle);
            }
        }
        return agent_paths;
    }
};


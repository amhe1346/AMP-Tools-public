import numpy as np
from collections import deque

class Agent:
    def __init__(self, q_init, q_goal, radius):
        self.q_init = np.array(q_init)
        self.q_goal = np.array(q_goal)
        self.radius = radius
        self.path = []

class DecentralPlanner:
    def __init__(self, workspace, obstacles, agents):
        self.workspace = workspace  # (x_min, x_max, y_min, y_max)
        self.obstacles = obstacles  # list of polygons or circles
        self.agents = agents        # list of Agent objects
        self.agent_paths = []

    def plan(self):
        dynamic_obstacles = list(self.obstacles)
        for agent in self.agents:
            path = self.goal_bias_rrt(agent, dynamic_obstacles)
            agent.path = path
            self.agent_paths.append(path)
            dynamic_obstacles.append(path)  # Add agent's path as obstacle for next agent
        return self.agent_paths

    def goal_bias_rrt(self, agent, obstacles):
        # Simple RRT with goal bias
        max_iter = 1000
        step_size = 0.5
        goal_sample_rate = 0.2
        tree = [agent.q_init]
        parent = {tuple(agent.q_init): None}
        for i in range(max_iter):
            if np.random.rand() < goal_sample_rate:
                sample = agent.q_goal
            else:
                sample = np.random.uniform(
                    [self.workspace[0], self.workspace[2]],
                    [self.workspace[1], self.workspace[3]]
                )
            nearest = min(tree, key=lambda n: np.linalg.norm(n - sample))
            direction = sample - nearest
            direction = direction / np.linalg.norm(direction) * step_size
            new_node = nearest + direction
            if not self.in_collision(new_node, obstacles, agent.radius):
                tree.append(new_node)
                parent[tuple(new_node)] = tuple(nearest)
                if np.linalg.norm(new_node - agent.q_goal) < step_size:
                    # Reached goal
                    return self.reconstruct_path(parent, new_node)
        return []  # Failed to find path

    def in_collision(self, point, obstacles, radius):
        # Placeholder: implement collision checking with obstacles and dynamic paths
        return False

    def reconstruct_path(self, parent, node):
        path = deque()
        while node is not None:
            path.appendleft(np.array(node))
            node = parent.get(tuple(node))
        return list(path)

# Example usage:
# workspace = (x_min, x_max, y_min, y_max)
# obstacles = [...]
# agents = [Agent(q_init, q_goal, radius), ...]
# planner = DecentralPlanner(workspace, obstacles, agents)
# paths = planner.plan()
